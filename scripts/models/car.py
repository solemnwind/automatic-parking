# scripts/models/car.py
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
        # plot
        self.rect = None

    def update(self, new_pose: Pose_t) -> None:
        self.pose = new_pose

    def patch(self, redraw: bool = True, color: str = 'blue', expand: bool = False) -> plt.Rectangle:
        x, y, phi = self.pose
        length = self.length
        width = self.width
        rear_to_base_axle = self.rear_to_base_axle
        linewidth = 1
        if expand:
            expansion = 0.03
            width += 2 * expansion
            length += 2 * expansion
            rear_to_base_axle += expansion
            linewidth *= 2
        left_bottom = (x - (rear_to_base_axle * np.cos(phi) - width / 2 * np.sin(phi)),
                       y - (rear_to_base_axle * np.sin(phi) + width / 2 * np.cos(phi)))

        if redraw:
            rect = patches.Rectangle(left_bottom, length, width,
                                     edgecolor=color, fill=None, linestyle="-", linewidth=linewidth,
                                     angle=phi * 180 / np.pi)
            return rect
        else:
            if self.rect is None:
                self.rect = patches.Rectangle(left_bottom, length, width,
                                              edgecolor=color, fill=None, linestyle="-", linewidth=linewidth,
                                              angle=phi * 180 / np.pi)
            else:
                self.rect.set_xy(left_bottom)
                self.rect.set_angle(phi * 180 / np.pi)
                self.rect.set_color(color)

            return self.rect

    def arrow(self, color: str = 'blue') -> patches.FancyArrow:
        arrow_length = self.wheelbase * 0.5
        arrow_dx = arrow_length * np.cos(self.pose[2])
        arrow_dy = arrow_length * np.sin(self.pose[2])

        arrow = patches.FancyArrow(self.pose[0], self.pose[1], arrow_dx, arrow_dy,
                                   color=color, alpha=0.5, width=0.2, head_width=0.5, head_length=0.5)

        return arrow
