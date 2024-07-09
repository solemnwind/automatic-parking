import numpy as np


Pose_t = tuple[float, float, float]
Idx_t = tuple[int, int, int]


def mod2pi(theta: float) -> float:
    """ theta -> (-pi, pi]"""
    phi = theta % (2 * np.pi)
    if phi > np.pi:
        phi -= 2 * np.pi
    elif phi <= -np.pi:
        phi += 2 * np.pi
    return phi


def discretize_angle(angle: float, angle_resolution: int) -> int:
    unit_angle = 2 * np.pi / angle_resolution
    idx_angle = int((angle + unit_angle / 2) % (2 * np.pi) // unit_angle)
    assert 0 <= idx_angle < angle_resolution
    return idx_angle


def recover_angle(index: int, angle_resolution: int) -> float:
    unit_angle = 2 * np.pi / angle_resolution
    phi = mod2pi(index * unit_angle)
    assert -np.pi < phi <= np.pi
    return phi


def addPose(pose: Pose_t, delta: Pose_t) -> Pose_t:
    return pose[0] + delta[0], pose[1] + delta[1], pose[2] + delta[2]
