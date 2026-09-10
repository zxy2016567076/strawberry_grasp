"""Pi scheduler + simulated MCU. Virtual time, no FSP or physical actuation."""
from enum import Enum
import numpy as np
from .protocol import MODEL, encode, decode, message
from .pressure import Sampler, classify
from .planning import plan_task


class State(str, Enum):
    IDLE = "IDLE"
    BELT_RUN = "BELT_RUN"
    BELT_STOP = "BELT_STOP"
    PRE_GRASP = "PRE_GRASP"
    GRASP = "GRASP"
    LIFT = "LIFT"
    PLACE = "PLACE"
    RETURN = "RETURN"


class Halt(RuntimeError):
    pass


class SimMCU:
    def __init__(self, arm):
        self.arm = arm
        self.q = arm.home.copy()
        self.gripper = arm.config["gripper"]["open"]
        self.belt = False
        self.armed = False
        self.fault = None
        self.last_seq = -1
        self.now_ms = 0
        self.last_rx_ms = 0
        self.watchdog_ms = 1000
        self.sampler = Sampler(arm.config["pressure_period_ms"])
        self.closing = False
        self.close_started = None
        self.grasp_samples = []
        self.contact = False
        self.holding = False
        self.next_grip_ms = 0
        self.last_step_ms = -arm.config["motion_period_ms"]
        self.command_count = 0
        self.executed_steps = 0

    def halt(self, reason):
        self.fault = reason
        self.belt = False
        self.armed = False
        self.closing = False
        # Freeze motion and gripper command; do not auto-home or drop the object.

    def receive(self, frame):
        try:
            m = decode(frame)
            seq, op = m["seq"], m["op"]
            if type(seq) is not int or seq <= self.last_seq:
                raise ValueError("stale sequence")
            if self.fault:
                raise ValueError("fault latched; instantiate/reset explicitly")
            if op == "hello":
                if self.armed:
                    raise ValueError("already negotiated")
                self.armed = True
            elif not self.armed:
                raise ValueError("V3 capability handshake required")
            elif op == "stop":
                self.halt("ESTOP")
            elif op == "step":
                q = self.arm.check_joints(m["joints_deg"])
                speed = m["speed"]
                c = self.arm.config
                cap = min(c["max_joint_step_deg"], c["speeds_deg_s"][speed] * c["motion_period_ms"] / 1000)
                if np.max(np.abs(q-self.q)) > cap + 1e-6:
                    raise ValueError("joint step/rate exceeded")
                if self.now_ms - self.last_step_ms < c["motion_period_ms"]:
                    raise ValueError("motion period violated")
                self.arm.check_workspace(self.arm.fk(q)[:3, 3])
                self.q = q
                self.executed_steps += 1
                self.last_step_ms = self.now_ms
            elif op == "belt":
                if type(m["enabled"]) is not bool:
                    raise ValueError("belt boolean required")
                self.belt = m["enabled"]
            elif op == "grip":
                if m["action"] == "open":
                    self.gripper = self.arm.config["gripper"]["open"]
                    self.closing = self.holding = False
                elif m["action"] == "close":
                    self.close_started = self.now_ms
                    self.next_grip_ms = self.now_ms
                    self.grasp_samples = []
                    self.closing = True
                    self.contact = self.holding = False
                else:
                    raise ValueError("invalid gripper action")
            elif op != "ping":
                raise ValueError("unknown operation")
            self.last_seq = seq
            self.last_rx_ms = self.now_ms
            self.command_count += 1
            return encode(message(seq, "ack", status="ok", motion_axes=6, gripper_axes=1))
        except (ValueError, KeyError, TypeError) as exc:
            self.halt(f"PROTOCOL: {exc}")
            raise Halt(self.fault) from exc

    def tick(self, now_ms, pressure_source):
        if now_ms < self.now_ms:
            raise ValueError("clock cannot go backwards")
        self.now_ms = now_ms
        value = self.sampler.tick(now_ms, pressure_source)
        g = self.arm.config["gripper"]
        if self.fault:
            return
        if self.armed and now_ms - self.last_rx_ms > self.watchdog_ms:
            self.halt("UART_WATCHDOG")
            return
        if value is not None and value >= g["overforce_delta"]:
            self.halt("OVERFORCE")
            return
        if self.closing:
            if value is not None:
                self.grasp_samples.append(value)
                self.contact |= value >= g["contact_delta"]
                if self.contact and value >= g["hold_delta"]:
                    self.holding = True
                    self.closing = False
            if now_ms - self.close_started >= g["timeout_ms"] and not self.holding:
                self.halt("NO_CONTACT_TIMEOUT")
            elif self.closing and now_ms >= self.next_grip_ms:
                self.gripper = max(g["closed"], self.gripper - g["step"])
                self.next_grip_ms = now_ms + g["period_ms"]


class Scheduler:
    def __init__(self, arm, fault=None):
        self.arm = arm
        self.mcu = SimMCU(arm)
        self.state = State.IDLE
        self.events = [{"ms": 0, "state": self.state.value}]
        self.seq = 0
        self.now = 0
        self.fault_injection = fault
        self.trace = []
        self.wire_example = None

    def transition(self, state):
        allowed = {State.IDLE: State.BELT_RUN, State.BELT_RUN: State.BELT_STOP,
                   State.BELT_STOP: State.PRE_GRASP, State.PRE_GRASP: State.GRASP,
                   State.GRASP: State.LIFT, State.LIFT: State.PLACE,
                   State.PLACE: State.RETURN, State.RETURN: State.IDLE}
        if allowed[self.state] != state:
            raise Halt("invalid state transition")
        self.state = state
        self.events.append({"ms": self.now, "state": state.value})

    def send(self, op, **fields):
        frame = encode(message(self.seq, op, **fields))
        self.wire_example = self.wire_example or frame.decode()
        reply = self.mcu.receive(frame)
        if self.fault_injection == "timeout" and op == "step":
            # The command may have executed; missing ACK must not be retried blindly.
            try:
                self.wait(self.mcu.watchdog_ms + 1, heartbeat=False)
            except Halt as exc:
                raise Halt(f"ACK_TIMEOUT: {exc}") from exc
            raise Halt("ACK_TIMEOUT")
        ack = decode(reply)
        if (ack.get("seq") != self.seq or ack.get("op") != "ack" or
                ack.get("status") != "ok" or ack.get("motion_axes") != 6 or
                ack.get("gripper_axes") != 1):
            raise Halt("capability/ACK mismatch")
        self.seq += 1

    def pressure_source(self, now):
        # Explicit synthetic sequence, no sensor or trained simulator.
        if self.mcu.close_started is None:
            return 0
        elapsed = now - self.mcu.close_started
        if self.fault_injection == "no-contact":
            return 20
        return min(650, max(0, elapsed - 30) * 5)

    def wait(self, duration, heartbeat=True):
        for _ in range(duration):
            self.now += 1
            self.mcu.tick(self.now, self.pressure_source)
            if self.fault_injection == "estop" and self.now >= 250:
                self.mcu.halt("ESTOP")
            if self.mcu.fault:
                raise Halt(self.mcu.fault)
            if heartbeat and self.now - self.mcu.last_rx_ms >= 200:
                self.send("ping")

    def move(self, name, motions):
        speed, steps = motions[name]
        for q in steps:
            self.wait(self.arm.config["motion_period_ms"])
            self.send("step", joints_deg=q.tolist(), speed=speed)
            self.trace.append({"ms": self.now, "waypoint": name, "speed": speed,
                               "joints_deg": self.mcu.q.tolist(),
                               "tcp_mm": self.arm.fk(self.mcu.q)[:3, 3].tolist()})

    def run(self, xyz, maturity):
        waypoints, ml = [], None
        try:
            # Full task preflight: no UART or belt action if any target/path fails.
            waypoints, motions = plan_task(self.arm, xyz, maturity)
            self.send("hello")
            self.transition(State.BELT_RUN)
            self.send("belt", enabled=True)
            self.wait(100)
            self.transition(State.BELT_STOP)
            self.send("belt", enabled=False)
            self.wait(100)
            self.transition(State.PRE_GRASP)
            self.send("grip", action="open")
            self.move("pre_grasp", motions)
            self.transition(State.GRASP)
            self.move("grasp", motions)
            self.send("grip", action="close")
            while self.mcu.closing:
                self.wait(1)
            if not self.mcu.holding:
                raise Halt("CONTACT_NOT_CONFIRMED")
            ml = classify(self.mcu.grasp_samples)
            self.transition(State.LIFT)
            self.move("lift", motions)
            self.transition(State.PLACE)
            self.move("transit", motions)
            self.move("place", motions)
            self.send("grip", action="open")
            self.transition(State.RETURN)
            self.move("retreat", motions)
            self.move("home", motions)
            self.transition(State.IDLE)
        except (ValueError, Halt) as exc:
            self.mcu.halt(str(exc))
        return {"status": "HALTED" if self.mcu.fault else "DONE",
                "fault": self.mcu.fault, "last_state": self.state.value,
                "belt_enabled": self.mcu.belt, "events": self.events,
                "waypoints": [{"name": w.name, "purpose": w.purpose,
                               "pose": w.target.tolist()} for w in waypoints],
                "motion_steps": len(self.trace), "virtual_duration_ms": self.now,
                "mcu_accepted_steps": self.mcu.executed_steps,
                "final_joints_deg": self.mcu.q.tolist(), "final_gripper": self.mcu.gripper,
                "sampling_target_ms": self.mcu.sampler.period_ms,
                "virtual_sample_count": self.mcu.sampler.count,
                "pressure_sequence": self.mcu.grasp_samples,
                "classification": ml, "trace": self.trace,
                "wire_example": self.wire_example}
