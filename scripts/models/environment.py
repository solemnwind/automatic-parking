# scripts/models/environment.py
# Defines 2D road environment with parking slots
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Environment:
    def __init__(self, scene: dict):
        self.resolution = scene["resolution"]
        self.name = scene['name']
        self.west = scene['west']
        self.east = scene['east']
        self.south = scene['south']
        self.north = scene['north']
        self.obstacles = scene['obstacles']
        self.slot = scene['slot']


    def draw(self, fig: plt.Figure = None, ax: plt.Axes = None) -> tuple[plt.Figure, plt.Axes]:
        if fig is None or ax is None:
            fig, ax = plt.subplots()

        ax.title.set_text(self.name)
        ax.set_xlim(self.west, self.east)
        ax.set_ylim(self.south, self.north)

        # Draw obstacles
        for obs in self.obstacles:
            rect = patches.Rectangle((obs[0], obs[2]), obs[1] - obs[0], obs[3] - obs[2],
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

        plt.gca().set_aspect('equal', adjustable='box')

        return fig, ax
