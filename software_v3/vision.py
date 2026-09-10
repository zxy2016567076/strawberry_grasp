"""Consume detection centers; example homography is not camera calibration."""
import numpy as np
from .kinematics import validate_pose


def target_to_base(config, detection):
    if detection["maturity"] not in config["bins_mm"]:
        raise ValueError("unknown maturity")
    uv = np.asarray(detection["pixel"], float)
    if uv.shape != (2,) or not np.isfinite(uv).all():
        raise ValueError("invalid pixel center")
    h = np.asarray(config["pixel_to_plane"], float)
    if h.shape != (3, 3) or not np.isfinite(h).all() or abs(np.linalg.det(h)) < 1e-12:
        raise ValueError("invalid homography")
    xyw = h @ np.r_[uv, 1]
    if abs(xyw[2]) < 1e-9:
        raise ValueError("homography maps to infinity")
    point = np.r_[xyw[:2] / xyw[2], config["grasp_z_mm"], 1]
    return (validate_pose(config["plane_to_base"]) @ point)[:3]


def from_detection(detection):
    """Adapter for vision/pi/detector.py Detection, no model import."""
    return {"pixel": list(detection.center), "maturity": detection.class_name,
            "source": "external detector; verify weights and calibration before hardware use"}
