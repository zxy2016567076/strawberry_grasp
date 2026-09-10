"""Configurable serial 6R FK and bounded numerical full-pose IK (mm, degrees)."""
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

DEFAULT_CONFIG = Path(__file__).parent / "config" / "example.json"


def load_config(path=DEFAULT_CONFIG):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def pose(xyz, rpy=(0, 90, 0)):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", rpy, degrees=True).as_matrix()
    t[:3, 3] = xyz
    return t


def validate_pose(t):
    t = np.asarray(t, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("invalid finite 4x4 pose")
    r = t[:3, :3]
    if not (np.allclose(t[3], [0, 0, 0, 1]) and
            np.allclose(r.T @ r, np.eye(3), atol=1e-7) and
            np.isclose(np.linalg.det(r), 1)):
        raise ValueError("invalid rigid transform")
    return t


class Unreachable(ValueError):
    pass


class Arm:
    def __init__(self, config):
        self.config = config
        for name in ("motion_period_ms", "pressure_period_ms"):
            if type(config[name]) is not int or config[name] <= 0:
                raise ValueError(f"{name} must be positive integer milliseconds")
        for value in [config["max_joint_step_deg"], config["cartesian_step_mm"],
                      *config["speeds_deg_s"].values()]:
            if not np.isfinite(value) or value <= 0:
                raise ValueError("finite positive motion parameters required")
        low, high = np.asarray(config["workspace_min_mm"]), np.asarray(config["workspace_max_mm"])
        if (low.shape != (3,) or high.shape != (3,) or not np.isfinite([low, high]).all()
                or np.any(low >= high) or not np.isfinite(config["safe_z_mm"])):
            raise ValueError("invalid workspace bounds")
        g = config["gripper"]
        if (not np.isfinite(list(g.values())).all() or
                not 0 <= g["closed"] < g["open"] <= 1 or g["step"] <= 0 or
                not 0 <= g["contact_delta"] < g["hold_delta"] < g["overforce_delta"] <= 65535 or
                g["period_ms"] <= 0 or g["timeout_ms"] <= 0):
            raise ValueError("invalid gripper configuration")
        joints = config["joints"]
        if len(joints) != 6 or len({j["name"] for j in joints}) != 6:
            raise ValueError("six distinct motion joints required; gripper is separate")
        self.axes = np.asarray([j["axis"] for j in joints], float)
        self.links = np.asarray([j["link_mm"] for j in joints], float)
        self.zero = np.asarray([j["zero_deg"] for j in joints], float)
        self.limits = np.asarray([j["limits_deg"] for j in joints], float)
        if (self.axes.shape != (6, 3) or self.links.shape != (6, 3) or
                self.limits.shape != (6, 2) or
                not all(np.isfinite(a).all() for a in [self.axes, self.links, self.zero, self.limits]) or
                np.any(np.linalg.norm(self.axes, axis=1) < 1e-9) or
                np.any(self.limits[:, 0] >= self.limits[:, 1])):
            raise ValueError("invalid joint configuration")
        self.axes /= np.linalg.norm(self.axes, axis=1)[:, None]
        self.home = self.check_joints(config["home_deg"])
        self.check_workspace(self.fk(self.home)[:3, 3])

    def check_joints(self, q):
        q = np.asarray(q, float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("expected six finite motion joint angles")
        if np.any(q < self.limits[:, 0]) or np.any(q > self.limits[:, 1]):
            raise Unreachable("joint limit exceeded")
        return q.copy()

    def fk(self, q):
        q = self.check_joints(q)
        t = np.eye(4)
        for axis, link, angle in zip(self.axes, self.links, q + self.zero):
            a = np.eye(4)
            a[:3, :3] = Rotation.from_rotvec(axis * np.deg2rad(angle)).as_matrix()
            a[:3, 3] = a[:3, :3] @ link
            t = t @ a
        return t

    def check_workspace(self, xyz):
        p = np.asarray(xyz, float)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError("invalid position")
        if (np.any(p < self.config["workspace_min_mm"]) or
                np.any(p > self.config["workspace_max_mm"]) or
                np.linalg.norm(p) > np.linalg.norm(self.links, axis=1).sum()):
            raise Unreachable("outside configured workspace")

    def error(self, q, target):
        actual = self.fk(q)
        return np.r_[actual[:3, 3] - target[:3, 3],
                     100 * Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec()]

    def ik(self, target, seed=None):
        target = validate_pose(target)
        self.check_workspace(target[:3, 3])
        seed = self.home if seed is None else self.check_joints(seed)
        # Deterministic local solve followed by alternate seeds, not exhaustive IK.
        seeds = [seed, self.home, self.limits.mean(axis=1)]
        seeds += [np.clip(seed + [0, d, -2*d, 20, d, -20],
                          self.limits[:, 0], self.limits[:, 1]) for d in (-30, 30)]
        for start in seeds:
            result = least_squares(self.error, start, args=(target,),
                                   bounds=(self.limits[:, 0], self.limits[:, 1]),
                                   max_nfev=160, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            e = self.error(result.x, target)
            if np.linalg.norm(e[:3]) < 0.05 and np.linalg.norm(e[3:]) / 100 < 1e-4:
                return self.check_joints(result.x)
        raise Unreachable("no full-pose IK solution found within limits / iteration budget")
