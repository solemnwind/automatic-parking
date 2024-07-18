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

        plt.gca().set_aspect('equal', adjustable='box')

        return fig, ax
