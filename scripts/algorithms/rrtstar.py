import numpy as np
from typing import Self
import logging
import time
# Import from "models" module
from .pathfinding import PathFindingAlgorithm
from models import get_reeds_shepp_path, interpolate_reeds_shepp_path, ReedsSheppPath
from models.utils import Pose_t

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class RRTNode:
    def __init__(self, pose: Pose_t, parent:Self or None,
                 cost_from_start: float, cost_from_parent: float, cost_to_end: float = np.inf,
                 path_from_parent: ReedsSheppPath or None = None):
        self.pose: Pose_t = pose
        self.cost_from_start = cost_from_start
        self.cost_from_parent = cost_from_parent
        self.cost_to_goal = cost_to_end
        self.parent: Self or None = parent
        self.children: list[Self] = []
        self.path_from_parent: ReedsSheppPath or None = path_from_parent

    def is_root(self) -> bool:
        return self.parent is None

    def get_path(self, step: float) -> list[Pose_t]:
        if self.parent is None:
            return [self.pose]

        path_samples_from_parent = [interpolate_reeds_shepp_path(self.parent.pose, self.path_from_parent, d + step)
                                    for d in np.arange(step, self.path_from_parent.distance, step)]
        return self.parent.get_path(step) + path_samples_from_parent


class RRTStar(PathFindingAlgorithm):
    def __init__(self, planner_params: dict, astar_params: dict, vehicle_params: dict):
        super().__init__(planner_params, astar_params, vehicle_params)
        self.rrt_root = RRTNode(self.start, None, 0, 0)
        self.nodes = [self.rrt_root]
        self.step = 0.2
        self.time_budget = 2000  # Maximum growing time in ms
        self.neighboring_range_square = 4
        self.goal_reached: bool = False
        self.goal_node: RRTNode or None = None

    def search(self):
        logger.info("RRT* Searching...")
        start_time = time.time()
        while not self.goal_reached:
            self.grow()

        elapsed_ms = round((time.time() - start_time) * 1000)
        logger.info("A path is found! Time elapsed: {} ms".format(elapsed_ms))
        return self.goal_node.get_path(self.step)

    def grow(self):
        # Sample new node
        current_sample: Pose_t
        while True:
            current_sample = self._uniform_sample()
            current_idx = tuple(self.occ_map.pose_to_index(current_sample))
            if not self.occ_map.has_collision(current_idx):
                break

        # Find best neighbor
        best_node : RRTNode or None = None
        best_cost_from_start: float = np.inf
        best_cost_from_parent: float = np.inf
        best_path_from_parent: ReedsSheppPath or None = None
        for candidate in self.nodes:
            # Check whether it's reachable from the neighbor
            path_neighbor_to_sample = get_reeds_shepp_path(candidate.pose, current_sample, self.radius)
            new_cost_from_start = path_neighbor_to_sample.distance + candidate.cost_from_start
            if new_cost_from_start >= best_cost_from_start:
                continue

            if self._is_path_passable(candidate.pose, path_neighbor_to_sample):
                best_node = candidate
                best_cost_from_start = new_cost_from_start
                best_cost_from_parent = path_neighbor_to_sample.distance
                best_path_from_parent = path_neighbor_to_sample

        if best_node is None:
            return

        new_node = RRTNode(current_sample, parent=best_node,
                           cost_from_start=best_cost_from_start, cost_from_parent=best_cost_from_parent,
                           path_from_parent=best_path_from_parent)
        best_node.children.append(new_node)

        # Reroute nodes in the neighboring area
        for neighbor in self.nodes:
            euclidean_distance = (neighbor.pose[0] - new_node.pose[0]) ** 2 + (neighbor.pose[1] - new_node.pose[1]) ** 2
            if euclidean_distance <= self.neighboring_range_square:
                path_sample_to_neighbor = get_reeds_shepp_path(current_sample, neighbor.pose, self.radius)
                cost_from_start_via_sample = new_node.cost_from_start + path_sample_to_neighbor.distance
                if cost_from_start_via_sample <= neighbor.cost_from_start and \
                    self._is_path_passable(current_sample, path_sample_to_neighbor):
                    neighbor.parent.children.remove(neighbor)
                    neighbor.parent = new_node
                    neighbor.path_from_parent = path_sample_to_neighbor
                    new_node.children.append(neighbor)

        self.nodes.append(new_node)

        # Check if the goal is reached
        if self.near_goal(current_sample):
            self.goal_reached = True
            self.goal_node = new_node

    def _uniform_sample(self) -> Pose_t:
        sample_goal = np.random.random() < 0.05
        if sample_goal:
            return self.goal

        sample = (self.env.west + np.random.random() * (self.env.east - self.env.west),
                  self.env.south + np.random.random() * (self.env.north - self.env.south),
                  -np.pi + np.random.random() * (2 * np.pi))
        return sample

    def _is_path_passable(self, start: Pose_t, path: ReedsSheppPath) -> bool:
        for d in np.arange(0, path.distance, self.step):
            pose = interpolate_reeds_shepp_path(start, path, d)
            idx = tuple(self.occ_map.pose_to_index(pose))
            if self.occ_map.has_collision(idx):
                return False
        return True