# scripts/models/environment.py
# Defines 2D road environment with parking slots
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import logging
# Import from modules
from .car import Car
from .utils import Pose_t, Idx_t, discretize_angle, recover_angle

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Environment:
    def __init__(self, scene: dict):
        self.origin = None
        self.collision_lut = None

        self.resolution = scene["resolution"]
        self.name = scene['name']
        self.west = scene['west']
        self.east = scene['east']
        self.south = scene['south']
        self.north = scene['north']
        self.obstacles = scene['obstacles']
        self.slot = scene['slot']

        self._init_occupancy_map()

    def _metric_to_index(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is
        the lesser index.
        """
        return np.floor((metric - self.origin)/self.resolution).astype('int')

    def _metric_to_index_range(self, west, east, south, north, outer_bound=True) -> tuple[int, int, int, int]:
        """
        A fast test that returns the closed index range intervals of voxels
        intercepting a rectangular bound. If outer_bound is true the returned
        index range is conservatively large, if outer_bound is false the index
        range is conservatively small.
        """

        sign = 1 if outer_bound else -1
        min_index_res = np.nextafter(self.resolution,  sign * np.inf)  # Use for lower corner.
        max_index_res = np.nextafter(self.resolution, -sign * np.inf)  # Use for upper corner.

        # Find minimum included index range.
        min_corner = np.array([west, south])
        min_frac_index = (min_corner - self.origin) / min_index_res
        min_index = np.floor(min_frac_index).astype('int')
        min_index[min_index == min_frac_index] -= 1
        min_index = np.maximum(0, min_index)
        # Find maximum included index range.
        max_corner = np.array([east, north])
        max_frac_index = (max_corner - self.origin) / max_index_res
        max_index = np.floor(max_frac_index).astype('int')
        max_index = np.minimum(max_index, np.asarray(self.map.shape)-1)
        return min_index[0], max_index[0], min_index[1], max_index[1]

    def _init_occupancy_map(self):
        dimensions_metric = np.array([self.east - self.west, self.north - self.south])
        self.map = np.zeros(np.ceil(dimensions_metric / self.resolution).astype('int'), dtype=bool)
        self.max_indices = self.map.shape
        self.origin = np.array([self.west, self.south])

        for obs in self.obstacles:
            w, e, s, n = self._metric_to_index_range(obs["west"], obs["east"], obs["south"], obs["north"],
                                                     outer_bound=True)
            self.map[w:e+1, s:n+1] = True

    def pose_to_index(self, pose: Pose_t, angle_resolution: int) -> Idx_t:
        # Discretize angles
        idx_angle = discretize_angle(pose[2], angle_resolution)
        idx_x, idx_y = self._metric_to_index(np.array(pose[:2]))
        return idx_x, idx_y, idx_angle

    def has_collision(self, idx: Idx_t, car: Car, angle_resolution: int) -> bool:
        if self.collision_lut is None:
            self.collision_lut = self._create_collision_lut(car, angle_resolution)

        origin = np.array(idx[:2])
        checking_points = origin + self.collision_lut[idx[2]]
        for idx in checking_points:
            if (idx[0] < 0 or idx[0] >= self.max_indices[0] or
                idx[1] < 0 or idx[1] >= self.max_indices[1] or
                    self.map[*idx]):
                return True

        return False

    def _create_collision_lut(self, car: Car, angle_resolution: int) -> list[list[np.ndarray]]:
        lut = []
        for i in range(angle_resolution):
            checking_points = []
            phi = recover_angle(i, angle_resolution)
            # Corners
            front_left = np.array([car.front_to_base_axle * np.cos(phi) - car.width / 2 * np.sin(phi),
                                   car.front_to_base_axle * np.sin(phi) + car.width / 2 * np.cos(phi)])
            front_right = np.array([car.front_to_base_axle * np.cos(phi) + car.width / 2 * np.sin(phi),
                                    car.front_to_base_axle * np.sin(phi) - car.width / 2 * np.cos(phi)])
            rear_left = np.array([-car.rear_to_base_axle * np.cos(phi) - car.width / 2 * np.sin(phi),
                                  -car.rear_to_base_axle * np.sin(phi) + car.width / 2 * np.cos(phi)])
            rear_right = np.array([-car.rear_to_base_axle * np.cos(phi) + car.width / 2 * np.sin(phi),
                                   -car.rear_to_base_axle * np.sin(phi) - car.width / 2 * np.cos(phi)])
            # Sample points on the edges
            for pair in [(front_left, front_right),
                         (front_right, rear_right),
                         (rear_right, rear_left),
                         (rear_left, front_left)]:
                for t in np.linspace(0, 1, 4, endpoint=False):
                    checking_points.append(self._metric_to_index(pair[0] * (1 - t) + pair[1] * t))

            checking_points.append(np.array((0, 0)))
            checking_points.append(self._metric_to_index(np.array((car.wheelbase * np.cos(phi),
                                                                   car.wheelbase * np.sin(phi)))))

            lut.append(checking_points)

        return lut

    def draw(self, fig: plt.Figure = None, ax: plt.Axes = None) -> tuple[plt.Figure, plt.Axes]:
        if fig is None or ax is None:
            fig, ax = plt.subplots()

        ax.title.set_text(self.name)
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

        plt.gca().set_aspect('equal', adjustable='box')

        return fig, ax
