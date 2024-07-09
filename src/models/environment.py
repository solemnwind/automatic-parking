# src/models/environment.py
# Defines 2D road environment with parking slots
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import toml
import logging
# Import from models module
# import car
# import utils
from .car import Car
from .utils import mod2pi, Pose_t, Idx_t

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Environment:
    def __init__(self, toml_file, resolution=(.1, .1), margin=.2):
        self.resolution = resolution
        self.margin = margin
        self.origin = None
        self._read_scene_config(toml_file)
        self._init_occupancy_map()

    def _read_scene_config(self, toml_file):
        with open(toml_file, 'r') as f:
            scene = toml.loads(f.read())

        bounds = scene['bounds']
        objects = scene['objects']
        self.name = scene['name']

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

    def index_to_metric_negative_corner(self, index):
        """
        Return the metric position of the most negative corner of a voxel, given its index in the occupancy grid
        """
        return index*np.array(self.resolution) + self.origin

    def index_to_metric_center(self, index):
        """
        Return the metric position of the center of a voxel, given its index in the occupancy grid
        """
        return self.index_to_metric_negative_corner(index) + self.resolution/2.0

    def metric_to_index(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is the lesser index.
        """
        return np.floor((metric - self.origin)/self.resolution).astype('int')

    def metric_to_index_ceil(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is the lesser index.
        """
        return np.ceil((metric - self.origin)/self.resolution).astype('int')

    def _metric_to_index_range(self, west, east, south, north, outer_bound=True) -> tuple[int, int, int, int]:
        """
        A fast test that returns the closed index range intervals of voxels
        intercepting a rectangular bound. If outer_bound is true the returned
        index range is conservatively large, if outer_bound is false the index
        range is conservatively small.
        """

        sign = 1 if outer_bound else -1
        min_index_res = np.nextafter(self.resolution,  sign * np.inf) # Use for lower corner.
        max_index_res = np.nextafter(self.resolution, -sign * np.inf) # Use for upper corner.

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
        self.origin = np.array([self.west, self.south])

        for obs in self.obstacles:
            w, e, s, n = self._metric_to_index_range(obs["west"], obs["east"], obs["south"], obs["north"],
                                                     outer_bound=True)
            self.map[w:e+1, s:n+1] = True

    def pose_to_index(self, pose: Pose_t, angle_resolution: int = 120) -> Idx_t:
        # Discretize angles
        unit_angle = 2 * np.pi / angle_resolution
        idx_angle = int((pose[2] + unit_angle / 2) % (2 * np.pi) // unit_angle)
        assert 0 <= idx_angle < angle_resolution
        idx_x, idx_y = self.metric_to_index(np.array(pose[:2]))
        return idx_x, idx_y, idx_angle

    def index_to_pose(self, index: Idx_t, angle_resolution: int = 120) -> Pose_t:
        # Recover angle
        unit_angle = 2 * np.pi / angle_resolution
        phi = mod2pi(index[2] * unit_angle)
        assert -np.pi < phi <= np.pi
        # Recover position
        return 0, 0, phi

    def has_collision(self, pose: Pose_t) -> bool:
        # TODO
        return False

    def draw(self):
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
        # plt.show()
        return fig, ax


if __name__ == '__main__':
    from pathlib import Path  # if you haven't already done so
    file = Path(__file__).resolve()
    config_file = file.parent.parent / 'utils/test_parking_lot.toml'
    env = Environment(config_file)
    env.draw()
