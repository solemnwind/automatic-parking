# src/models/environment.py
# Defines 2D road environment with parking slots
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import toml
from models.car import Car
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Environment:
    def __init__(self, toml_file):
        self.east = None
        self.west = None
        self.south = None
        self.north = None
        self.obstacles = None
        self.slot = None
        self.car = None
        self.read_scene_config(toml_file)

    def read_scene_config(self, toml_file):
        with open(toml_file, 'r') as f:
            scene = toml.loads(f.read())

        bounds = scene['bounds']
        objects = scene['objects']

        # Read bounds
        self.west = bounds['west']
        self.east = bounds['east']
        self.south = bounds['south']
        self.north = bounds['north']

        # Read objects
        self.obstacles = objects['obstacles']
        self.slot = objects['slot']
        car_params = objects['car']
        dimensions = car_params['dimensions']
        if isinstance(dimensions, bool):
            dimensions = None
        self.car = Car(pos=car_params['position'],
                       ori_deg=car_params['orientation'],
                       dim=dimensions,
                       max_speed=car_params['max_speed'],
                       max_accel=car_params['max_accel'],
                       max_steering_angle_deg=car_params['max_steering_angle'])

        logger.info('Read scene config: %s', toml_file)

    def draw(self):
        fig, ax = plt.subplots()
        ax.set_xlim(self.west, self.east)
        ax.set_ylim(self.south, self.north)

        # Draw obstacles
        for obs in self.obstacles:
            rect = patches.Rectangle((obs["west"], obs["south"]),
                                     obs["east"] - obs["west"], obs["north"] - obs["south"],
                                     facecolor='black', alpha=0.7, edgecolor=None)
            ax.add_patch(rect)
            rect.set_clip_path(rect)

        # Draw parking slot
        if self.slot:
            slot = self.slot
            rect = patches.Rectangle((slot["position"][0], slot["position"][1]),
                                     slot["width"], slot["length"],
                                     edgecolor='gold', fill=None, linestyle="--", linewidth=3,
                                     angle=slot["orientation"]-90)
            ax.add_patch(rect)
            rect.set_clip_path(rect)

            # Calculate the end point of the arrow
            arrow_length = slot["length"] / 3
            orientation_rad = np.deg2rad(slot["orientation"])
            arrow_dx = arrow_length * np.cos(orientation_rad)
            arrow_dy = arrow_length * np.sin(orientation_rad)

            # Draw arrow indicating slot orientation
            theta = orientation_rad - np.pi / 2
            slot_center_x = (slot["width"] * np.cos(theta) - slot["length"] * np.sin(theta)) / 2 + slot["position"][0]
            slot_center_y = (slot["length"] * np.cos(theta) + slot["width"] * np.sin(theta)) / 2 + slot["position"][1]

            arrow = patches.FancyArrow(slot_center_x - arrow_dx, slot_center_y - arrow_dy,
                                       arrow_dx, arrow_dy,
                                       width=0.25, head_width=0.7, head_length=0.8, color='gold', alpha=0.6)
            ax.add_patch(arrow)

        # Draw car
        if self.car:
            car = self.car
            rect = patches.Rectangle(car.pos, car.width, car.length, color='blue', alpha=0.5, angle=car.ori-90)
            ax.add_patch(rect)

        plt.gca().set_aspect('equal', adjustable='box')
        plt.show()


if __name__ == '__main__':
    config_file = '../utils/test_parking_lot.toml'
    env = Environment(config_file)
    env.draw()
