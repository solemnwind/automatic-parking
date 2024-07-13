import numpy as np

Pose_t = tuple[float, float, float]
Idx_t = tuple[int, int, int]


def mod2pi(theta: float) -> float:
    """
    Convert the angle to the range of (-pi, pi]
    :param theta: (radian)
    :return: angle within (-pi, pi]
    """
    phi = theta % (2 * np.pi)
    if phi > np.pi:
        phi -= 2 * np.pi
    elif phi <= -np.pi:
        phi += 2 * np.pi
    return phi


def discretize_angle(angle: float, angle_resolution: int) -> int:
    """
    Compute the discrete index of a given angle in radian.
    :param angle: (radian)
    :param angle_resolution: number of angle partitions
    :return: the index corresponding to the angle
    """
    unit_angle = 2 * np.pi / angle_resolution
    idx_angle = int((angle + unit_angle / 2) % (2 * np.pi) // unit_angle)
    assert 0 <= idx_angle < angle_resolution
    return idx_angle


def recover_angle(index: int, angle_resolution: int) -> float:
    """
    Compute the angle from an index.
    :param index: N
    :param angle_resolution: number of angle partitions
    :return: the center angle of indexed partition
    """
    unit_angle = 2 * np.pi / angle_resolution
    phi = mod2pi(index * unit_angle)
    assert -np.pi < phi <= np.pi
    return phi


def addPose(pose: Pose_t, delta: Pose_t) -> Pose_t:
    return pose[0] + delta[0], pose[1] + delta[1], pose[2] + delta[2]
