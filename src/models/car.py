# src/models/car.py
# Defines 2D car model
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from .utils import Pose_t


class Car:
    def __init__(self, vehicle_params: dict):
        # pose
        self.pose: Pose_t or None = None
        # dimensions
        self.length = vehicle_params["length"]
        self.width = vehicle_params["width"]
        self.wheelbase = vehicle_params["wheelbase"]
        self.front_to_base_axle = vehicle_params["front_to_base_axle"]
        self.rear_to_base_axle = self.length - self.front_to_base_axle
        # performance
        max_steering_angle_deg = vehicle_params["max_steering_angle"]
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
                                 color='blue', alpha=0.2,
                                 angle=phi * 180 / np.pi)
        ax.add_patch(rect)

        return fig, ax
