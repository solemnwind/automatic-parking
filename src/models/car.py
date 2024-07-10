# src/models/car.py
# Defines 2D car model
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from .utils import mod2pi, Pose_t

default_car_dimension: dict[str, float] = {"length": 2, "width": 1, "wheelbase": 1.4, "front_to_base_axle": 1.6}


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
        self.pose: Pose_t = (pos[0], pos[1], ori_deg * np.pi / 180)
        # dimensions
        if dim is None:
            dim = default_car_dimension
        self.length = dim["length"]
        self.width = dim["width"]
        self.wheelbase = dim["wheelbase"]
        self.front_to_base_axle = dim["front_to_base_axle"]
        self.rear_to_base_axle = self.length - self.front_to_base_axle
        # performance
        self.max_speed = max_speed
        self.max_accel = max_accel
        assert 0 < max_steering_angle_deg < 90, "Max steering angle degree must be less than 90 degrees."
        self.max_steering_angle = max_steering_angle_deg * np.pi / 180
        self.minimum_turning_radius = self.wheelbase / np.tan(self.max_steering_angle)

    def update(self, new_pose: Pose_t) -> None:
        self.pose = new_pose

    def draw(self, fig: plt.Figure = None, ax: plt.Axes = None) -> tuple[plt.Figure, plt.Axes]:
        if fig is None or ax is None:
            fig, ax = plt.subplots(111)

        x, y, phi = self.pose
        left_bottom = (x - (self.rear_to_base_axle * np.cos(phi) - self.width / 2 * np.sin(phi)),
                       y - (self.rear_to_base_axle * np.sin(phi) + self.width / 2 * np.cos(phi)))

        rect = patches.Rectangle(left_bottom, self.length, self.width,
                                 color='blue', alpha=0.3,
                                 angle=phi * 180 / np.pi)
        ax.add_patch(rect)

        return fig, ax
