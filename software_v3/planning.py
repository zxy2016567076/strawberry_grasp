"""Task waypoints are data; scheduler states live in runtime.py."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .kinematics import pose, Unreachable


@dataclass
class Waypoint:
    name: str
    purpose: str
    target: np.ndarray


def task_waypoints(arm, xyz, maturity):
    c = arm.config
    arm.check_workspace(xyz)
    destination = c["bins_mm"][maturity]
    z = c["safe_z_mm"]
    if xyz[2] >= z or destination[2] >= z:
        raise Unreachable("grasp/place must be below transit clearance")
    rpy = c["tool_rpy_deg"]
    return [
        Waypoint("home", "start and return reference", arm.fk(arm.home)),
        Waypoint("pre_grasp", "safe height above target", pose([*xyz[:2], z], rpy)),
        Waypoint("grasp", "slow vertical approach, then pressure close", pose(xyz, rpy)),
        Waypoint("lift", "vertical lift before lateral transfer", pose([*xyz[:2], z], rpy)),
        Waypoint("transit", "transfer at clearance above selected bin", pose([*destination[:2], z], rpy)),
        Waypoint("place", "slow descent, then open gripper", pose(destination, rpy)),
        Waypoint("retreat", "lift clear of bin before return home", pose([*destination[:2], z], rpy)),
    ]


def joint_steps(arm, start, end, speed):
    c = arm.config
    start, end = arm.check_joints(start), arm.check_joints(end)
    increment = min(c["max_joint_step_deg"],
                    c["speeds_deg_s"][speed] * c["motion_period_ms"] / 1000)
    if increment <= 0:
        raise ValueError("positive interpolation rate required")
    count = max(1, math.ceil(float(np.max(np.abs(end - start))) / increment))
    return [start + (end - start) * (i / count) for i in range(1, count + 1)]


def segment(arm, start, target, speed):
    """Cartesian samples -> seeded IK -> bounded joint steps, checked before execution.

    Checks TCP floor and transit clearance only; NOT self/environment collision checking.
    """
    c = arm.config
    initial = arm.fk(start)
    arm.check_workspace(target[:3, 3])
    lateral = np.linalg.norm(initial[:2, 3] - target[:2, 3]) > 0.1
    floor = c["safe_z_mm"] if lateral else min(initial[2, 3], target[2, 3])
    if lateral and min(initial[2, 3], target[2, 3]) < floor - 0.1:
        raise Unreachable("lateral transfer below safety clearance")
    count = max(1, math.ceil(np.linalg.norm(target[:3, 3] - initial[:3, 3]) / c["cartesian_step_mm"]))
    rotation = Slerp([0, 1], Rotation.from_matrix(np.stack([initial[:3, :3], target[:3, :3]])))
    q, steps = np.asarray(start, float), []
    for f in np.linspace(0, 1, count + 1)[1:]:
        t = np.eye(4)
        t[:3, 3] = initial[:3, 3] * (1-f) + target[:3, 3] * f
        t[:3, :3] = rotation(f).as_matrix()
        goal = arm.ik(t, q)
        for sample in joint_steps(arm, q, goal, speed):
            tcp = arm.fk(sample)[:3, 3]
            arm.check_workspace(tcp)
            if tcp[2] < floor - 0.1:
                raise Unreachable("interpolated TCP violates clearance")
            steps.append(sample)
        q = goal
    return steps


def plan_task(arm, xyz, maturity):
    waypoints = task_waypoints(arm, xyz, maturity)
    q, motions = arm.home, {}
    for w, speed in [(w, "slow" if w.name in ("grasp", "place") else "fast")
                     for w in waypoints[1:]] + [(waypoints[0], "fast")]:
        steps = segment(arm, q, w.target, speed)
        motions[w.name] = (speed, steps)
        q = steps[-1]
    return waypoints, motions
