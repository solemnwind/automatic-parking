import numpy as np

# Import from modules
from .car import Car
from .utils import Pose_t, Idx_t, discretize_angle, recover_angle


class OccupancyMap:
    def __init__(self, bounds: tuple[float, float, float, float], obstacles: list[dict[str, int]], resolution: float):
        self.resolution = resolution
        self.collision_lut = None

        dimensions_metric = np.array([bounds[1] - bounds[0], bounds[3] - bounds[2]])
        self.map = np.zeros(np.ceil(dimensions_metric / self.resolution).astype('int'), dtype=bool)
        self.max_indices = self.map.shape
        self.origin = np.array([bounds[0], bounds[2]])

        for obs in obstacles:
            w, e, s, n = self._metric_to_index_range(obs["west"], obs["east"], obs["south"], obs["north"],
                                                     outer_bound=True)
            self.map[w:e+1, s:n+1] = True

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

    def _metric_to_index(self, metric):
        """
        Returns the index of the voxel containing a metric point.
        Remember that this and index_to_metric and not inverses of each other!
        If the metric point lies on a voxel boundary along some coordinate,
        the returned index is
        the lesser index.
        """
        return np.floor((metric - self.origin)/self.resolution).astype('int')

    def pose_to_index(self, pose: Pose_t, angle_resolution: int) -> Idx_t:
        # Discretize angles
        idx_angle = discretize_angle(pose[2], angle_resolution)
        idx_x, idx_y = self._metric_to_index(np.array(pose[:2]))
        return idx_x, idx_y, idx_angle

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
