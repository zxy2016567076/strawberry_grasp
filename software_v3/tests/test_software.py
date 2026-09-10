import copy
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from software_v3.kinematics import Arm, Unreachable, load_config, pose
from software_v3.planning import joint_steps, plan_task, segment
from software_v3.pressure import Sampler, prepare_input, classify, load_weights
from software_v3.protocol import FrameStream, encode, decode, message
from software_v3.runtime import Halt, Scheduler, SimMCU, State
from software_v3.vision import target_to_base, from_detection


@pytest.fixture
def arm():
    return Arm(load_config())


def test_fk_zero_and_configurable_zero_offset(arm):
    np.testing.assert_allclose(arm.fk(np.zeros(6)), pose([360, 0, 120], [0, 0, 0]), atol=1e-9)
    cfg = copy.deepcopy(arm.config)
    cfg["joints"][0]["zero_deg"] = 10
    changed = Arm(cfg)
    q = arm.home.copy()
    q[0] = 10
    np.testing.assert_allclose(changed.fk(arm.home), arm.fk(q), atol=1e-9)


def test_six_independent_motion_axes_and_no_gripper_in_fk(arm):
    q = np.array([5, -50, 65, 20, 40, 15.])
    t = arm.fk(q)
    jac = np.column_stack([(arm.error(q + np.eye(6)[i] * 1e-3, t) / 1e-3) for i in range(6)])
    assert np.linalg.matrix_rank(jac, tol=1e-5) == 6
    other = q.copy()
    other[5] += 20
    assert not np.allclose(arm.fk(other)[:3, :3], t[:3, :3])
    with pytest.raises(ValueError):
        arm.fk([*q, 0.5])


@pytest.mark.parametrize("seed", [1, 7, 23])
def test_full_pose_ik_roundtrip(arm, seed):
    rng = np.random.default_rng(seed)
    for _ in range(5):
        q = np.array([0, -50, 65, 20, 40, 15]) + rng.uniform(-8, 8, 6)
        t = arm.fk(q)
        solved = arm.ik(t, arm.home)
        actual = arm.fk(solved)
        assert np.linalg.norm(actual[:3, 3]-t[:3, 3]) < .05
        assert np.linalg.norm(Rotation.from_matrix(t[:3, :3] @ actual[:3, :3].T).as_rotvec()) < 1e-4


@pytest.mark.parametrize("bad", [[1000, 0, 80], [220, 0, 0], [float('nan'), 0, 80]])
def test_workspace_rejects(arm, bad):
    with pytest.raises(ValueError):
        arm.ik(pose(bad))


def test_ik_rejects_geometrically_unreachable_inside_box(arm):
    with pytest.raises(Unreachable):
        arm.ik(pose([350, 200, 340]))


def test_limits_and_invalid_rotation(arm):
    with pytest.raises(Unreachable):
        arm.fk([171, 0, 0, 0, 0, 0])
    t = pose([220, 20, 70]); t[0, 0] = 5
    with pytest.raises(ValueError):
        arm.ik(t)
    cfg = copy.deepcopy(arm.config); cfg["joints"] = cfg["joints"][:5]
    with pytest.raises(ValueError):
        Arm(cfg)


@pytest.mark.parametrize("key,value", [("motion_period_ms", 0), ("pressure_period_ms", -5),
                                      ("cartesian_step_mm", float("nan"))])
def test_invalid_timing_configuration(key, value):
    config = load_config(); config[key] = value
    with pytest.raises(ValueError):
        Arm(config)


def test_coordinate_chain_and_real_detection_adapter(arm):
    c = copy.deepcopy(arm.config)
    c["plane_to_base"][0][3] = 10
    target = {"pixel": [320, 240], "maturity": "ripe"}
    np.testing.assert_allclose(target_to_base(c, target), [230, 20, 70])
    class Detection:
        center = (320, 240)
        class_name = "ripe"
    assert from_detection(Detection())["pixel"] == [320, 240]
    with pytest.raises(ValueError):
        target_to_base(c, {**target, "maturity": "unknown"})
    c["pixel_to_plane"] = [[1, 0, 0], [0, 1, 0], [0, 0, 0]]
    with pytest.raises(ValueError):
        target_to_base(c, target)


@pytest.mark.parametrize("maturity", ["ripe", "semi_ripe", "unripe"])
def test_seven_waypoints_all_bins_and_clearance(arm, maturity):
    wp, motions = plan_task(arm, [220, 20, 70], maturity)
    assert [w.name for w in wp] == ["home", "pre_grasp", "grasp", "lift", "transit", "place", "retreat"]
    q = arm.home
    for name in ["pre_grasp", "grasp", "lift", "transit", "place", "retreat", "home"]:
        speed, samples = motions[name]
        cap = min(1, arm.config["speeds_deg_s"][speed] * .02)
        for sample in samples:
            assert np.max(np.abs(sample - q)) <= cap + 1e-8
            tcp = arm.fk(sample)[:3, 3]
            arm.check_workspace(tcp)
            if name in ("pre_grasp", "transit", "home"):
                assert tcp[2] >= arm.config["safe_z_mm"] - .1
            q = sample
    np.testing.assert_allclose(arm.fk(q), wp[0].target, atol=.05)


def test_fast_slow_and_low_transfer_rejected(arm):
    q = arm.home.copy(); q[0] += 15
    assert len(joint_steps(arm, arm.home, q, "slow")) > len(joint_steps(arm, arm.home, q, "fast"))
    start = arm.ik(pose([220, 20, 70]))
    with pytest.raises(Unreachable, match="lateral"):
        segment(arm, start, pose([200, 80, 70]), "fast")


def test_protocol_roundtrip_fragmentation_and_legacy_wire_isolation():
    m = message(0, "hello")
    frame = encode(m)
    assert not any(65 <= b <= 90 for b in frame)
    assert decode(frame) == m
    stream = FrameStream()
    assert stream.feed(frame[:7]) == []
    assert stream.feed(frame[7:] + frame) == [frame, frame]
    with pytest.raises(ValueError):
        stream.feed(b"a" * 2049)


@pytest.mark.parametrize("frame", [b"A\n", b"K 90 90 90 90 90 80\n", b"~00:00000000\n",
                                    encode({"v": 2, "model": "six-servos"}), b"~0:00000000\n"])
def test_protocol_rejects_legacy_and_corrupt(frame):
    with pytest.raises(ValueError):
        decode(frame)


def test_mcu_requires_handshake_replay_and_independent_gripper(arm):
    m = SimMCU(arm)
    with pytest.raises(Halt, match="handshake"):
        m.receive(encode(message(0, "step", joints_deg=arm.home.tolist(), speed="fast")))
    m = SimMCU(arm)
    m.receive(encode(message(0, "hello")))
    before = m.q.copy()
    m.receive(encode(message(1, "grip", action="close")))
    for ms in range(200):
        m.tick(ms, lambda t: min(t*5, 650))
    assert m.contact and m.holding and not m.closing
    np.testing.assert_array_equal(m.q, before)
    with pytest.raises(Halt, match="sequence"):
        m.receive(encode(message(1, "ping")))


def test_receiver_rejects_large_step_and_watchdog(arm):
    m = SimMCU(arm); m.receive(encode(message(0, "hello")))
    q = arm.home.copy(); q[0] += 10
    with pytest.raises(Halt, match="step/rate"):
        m.receive(encode(message(1, "step", joints_deg=q.tolist(), speed="fast")))
    np.testing.assert_array_equal(m.q, arm.home)
    m = SimMCU(arm); m.receive(encode(message(0, "hello")))
    m.tick(1001, lambda _: 0)
    assert m.fault == "UART_WATCHDOG" and not m.belt


def test_sampling_decoupled_from_slow_motion_and_no_backfill(arm):
    m = SimMCU(arm); m.receive(encode(message(0, "hello")))
    for ms in range(101):
        m.tick(ms, lambda _: 10)
        if ms % 20 == 0:
            m.receive(encode(message(ms+1, "step", joints_deg=arm.home.tolist(), speed="slow")))
    assert [t for t, _ in m.sampler.samples] == list(range(0, 101, 5))
    s = Sampler(5); s.tick(0, lambda _: 1); s.tick(23, lambda _: 1)
    assert list(s.samples) == [(0, 1), (23, 1)]


def test_overforce_and_explicit_stop_latch(arm):
    for action in ["overforce", "stop"]:
        m = SimMCU(arm); m.receive(encode(message(0, "hello")))
        if action == "overforce":
            m.tick(0, lambda _: 1200)
        else:
            m.receive(encode(message(1, "stop")))
        assert m.fault in ["OVERFORCE", "ESTOP"]
        with pytest.raises(Halt):
            m.receive(encode(message(2, "hello")))


@pytest.mark.parametrize("fault,reason", [("estop", "ESTOP"), ("timeout", "UART_WATCHDOG"),
                                         ("no-contact", "NO_CONTACT_TIMEOUT"), ("unreachable", "workspace")])
def test_failure_exits_no_belt_no_restart(arm, fault, reason):
    scheduler = Scheduler(arm, fault)
    r = scheduler.run([1000, 0, 70] if fault == "unreachable" else [220, 20, 70], "ripe")
    assert r["status"] == "HALTED" and reason in r["fault"] and not r["belt_enabled"]
    assert not scheduler.mcu.armed
    frozen = scheduler.mcu.q.copy()
    scheduler.mcu.tick(scheduler.now+5000, lambda _: 0)
    np.testing.assert_array_equal(frozen, scheduler.mcu.q)
    if fault == "unreachable":
        assert scheduler.mcu.command_count == 0
    if fault == "timeout":
        assert r["fault"].startswith("ACK_TIMEOUT")
        assert r["mcu_accepted_steps"] == 1  # command accepted, ACK lost; no blind retry


def test_demo_eight_states_and_mlp_diagnostic_only(arm, monkeypatch):
    import software_v3.runtime as runtime
    monkeypatch.setattr(runtime, "classify", lambda _: {"class": "OVERFORCE", "note": "injected diagnostic"})
    r = Scheduler(arm).run([220, 20, 70], "ripe")
    assert r["status"] == "DONE" and r["last_state"] == "IDLE"
    assert [e["state"] for e in r["events"]] == [s.value for s in State] + ["IDLE"]
    assert r["classification"]["class"] == "OVERFORCE"  # no classifier-to-gripper feedback
    assert not r["belt_enabled"]


def test_mlp_preprocess_and_parameter_count():
    np.testing.assert_allclose(prepare_input([100, 200])[-2:], [.1, .2])
    assert np.count_nonzero(prepare_input([100, 200])[:-2]) == 0
    np.testing.assert_allclose(prepare_input(range(32)), np.arange(16, 32)/1000)
    assert sum(a.size for a in load_weights()) == 187
    with pytest.raises(ValueError):
        prepare_input([-1])


def test_c_python_mlp_parity(tmp_path):
    compiler = shutil.which("gcc") or shutil.which("cc")
    if compiler is None:
        pytest.skip("host C compiler unavailable")
    root = Path(__file__).resolve().parents[2]
    source = tmp_path / "mlp.c"
    source.write_text('''#include <stdint.h>
#include <stdio.h>
#include "tinyml_grasp.h"
int main(void) {
  uint16_t log[40]; unsigned n;
  while (scanf("%u", &n)==1) {
    if(n>40) return 2;
    for(unsigned i=0;i<n;i++){unsigned v; if(scanf("%u", &v)!=1) return 3; log[i]=(uint16_t)v;}
    float input[16], score;
    tinyml_prepare_input(log, (uint16_t)n, input);
    int label=(int)tinyml_classify(input,&score);
    printf("%d %.8f\\n", label, score);
  }
  return 0;
}
''', encoding="utf-8")
    executable = tmp_path / "mlp.exe"
    subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-I", str(root / "mcu"),
                    str(source), "-o", str(executable)], check=True, capture_output=True, text=True)
    rng = np.random.default_rng(42)
    sequences = [[], [100, 200], [0]*16, [650]*16, [1500]*16]
    sequences += [rng.integers(0, 2000, size=n).tolist() for n in range(1, 33)]
    data = "\n".join(f"{len(s)} " + " ".join(map(str, s)) for s in sequences) + "\n"
    lines = subprocess.run([str(executable)], input=data, text=True, capture_output=True, check=True).stdout.splitlines()
    assert len(lines) == len(sequences)
    for line, values in zip(lines, sequences):
        label, logit = line.split()
        result = classify(values)
        assert int(label) == int(np.argmax(result["logits"]))
        assert float(logit) == pytest.approx(max(result["logits"]), abs=2e-5)
