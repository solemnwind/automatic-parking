# src/models/car.py
# Defines 2D car model
import numpy as np
# import utils
from .utils import mod2pi, Pose_t

default_car_dimension: dict[str, float] = {"length": 2, "width": 1, "wheelbase": 1.4, "front_to_rear_axle": 1.6}


def deg2rad(deg: float) -> float:
    return mod2pi(deg * np.pi / 180)


class Car:
    def __init__(self,
                 pos: tuple[float, float],
                 ori_deg: float,
                 dim: dict[str, float] = None,
                 max_speed: float = 2.0,
                 max_accel: float = 2.0,
                 max_steering_angle_deg: float = 35.0):
        """
        Initialize Car with position, orientation and dimensions.

        :param tuple[float, float] pos: Car's position (m)
        :param float ori_deg: Car's orientation (degrees)
        :param dict[str, float] dim: Car's dimensions (m)
        :param float max_speed: Car's maximum speed (m/s)
        :param float max_accel: Car's maximum acceleration (m/s^2)
        :param float max_steering_angle_deg: Car's maximum steering angle (degrees)
        """
        # pose
        self.pos = pos
        self.ori = ori_deg
        self.pose: Pose_t = (pos[0], pos[1], ori_deg * np.pi / 180)
        # dimensions
        if dim is None:
            dim = default_car_dimension
        self.length = dim["length"]
        self.width = dim["width"]
        self.wheelbase = dim["wheelbase"]
        self.front_to_rear_axle = dim["front_to_rear_axle"]
        self.back_to_rear_axle = self.length - self.front_to_rear_axle
        # performance
        self.max_speed = max_speed
        self.max_accel = max_accel
        assert 0 < max_steering_angle_deg < 90, "Max steering angle degree must be less than 90 degrees."
        self.max_steering_angle = max_steering_angle_deg * np.pi / 180
        self.minimum_turning_radius = self.wheelbase / np.tan(self.max_steering_angle)

    def get_turning_radius(self, theta: float) -> float:
        if abs(theta) > self.max_steering_angle:
            raise ValueError("Turning radius exceeds maximum turning radius.")
        return self.wheelbase / np.tan(abs(theta))


if __name__ == '__main__':
    car1 = Car((1, 0), 90)
