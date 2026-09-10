"""python -m software_v3.demo --out software_v3/artifacts"""
import argparse
import json
from pathlib import Path
from .kinematics import Arm, load_config, DEFAULT_CONFIG
from .runtime import Scheduler
from .vision import target_to_base


def render_report(result, output):
    template = (Path(__file__).parent / "report.html").read_text(encoding="utf-8")
    payload = json.dumps(result, ensure_ascii=False, allow_nan=False).replace("<", "\\u003c")
    output.write_text(template.replace("__DATA__", payload), encoding="utf-8")


def main():
    p = argparse.ArgumentParser(description="Six-axis software-only sorting demonstration")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument("--target", type=Path, default=Path(__file__).parent / "examples" / "target.json")
    p.add_argument("--fault", choices=["timeout", "unreachable", "estop", "no-contact"])
    p.add_argument("--out", type=Path, default=Path("software_v3/artifacts"))
    args = p.parse_args()
    config = load_config(args.config)
    detection = json.loads(args.target.read_text(encoding="utf-8"))
    xyz = target_to_base(config, detection)
    if args.fault == "unreachable":
        xyz[0] = 1000
    result = Scheduler(Arm(config), args.fault).run(xyz, detection["maturity"])
    result.update({"provenance": "SOFTWARE SIMULATION: synthetic detection, geometry, calibration and pressure. Not hardware evidence.",
                   "detection": detection, "target_base_mm": xyz.tolist(), "config": config})
    args.out.mkdir(parents=True, exist_ok=True)
    stem = args.fault or "demo"
    (args.out / f"{stem}.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    render_report(result, args.out / f"{stem}.html")
    print(json.dumps({k: result[k] for k in ["status", "fault", "motion_steps", "virtual_duration_ms", "virtual_sample_count"]}))
    print(f"Report: {args.out / (stem + '.html')}")
    return 0 if result["status"] == "DONE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
