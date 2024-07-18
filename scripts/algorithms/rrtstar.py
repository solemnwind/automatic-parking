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
                 cost_from_start: float, cost_from_parent: float, cost_to_goal: float = np.inf,
                 path_from_parent: ReedsSheppPath or None = None):
        self.pose: Pose_t = pose
        self.cost_from_start = cost_from_start
        self.cost_from_parent = cost_from_parent
        self.cost_to_goal = cost_to_goal
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
    def __init__(self, planner_params: dict, rrtstar_params: dict, vehicle_params: dict):
        super().__init__(planner_params, vehicle_params)
        self.step = rrtstar_params["step_length"]
        self.time_budget = rrtstar_params["time_budget"]  # Maximum growing time in ms
        self.neighboring_range_square = rrtstar_params["neighboring_range"] ** 2

        self.rrt_root = RRTNode(self.start, None, 0, 0)
        self.nodes = {self.rrt_root}
        self.goal_reached: bool = False
        self.goal_node = RRTNode(self.goal, None, np.inf, np.inf, 0)

    def search(self) -> list[Pose_t] | None:
        logger.info("RRT* Searching...")
        start_time = time.time()

        goal_idx = tuple(self.occ_map.pose_to_index(self.goal))
        if self.occ_map.has_collision(goal_idx):
            logger.warning("The goal is unreachable!")
            return None

        time_budget_s = self.time_budget / 1e3
        while time.time() - start_time < time_budget_s:
            self.grow()

        elapsed_ms = round((time.time() - start_time) * 1000)
        if self.goal_reached:
            logger.info("A path is found! Time elapsed: {} ms".format(elapsed_ms))
            return self.goal_node.get_path(self.step)
        else:
            logger.info("No path is found within the time budget of {} ms!".format(self.time_budget))
            return None

    def grow(self):
        # Sample new node
        sample = self._sample_new_pose()

        # Find best parent node
        best_parent, best_cost_from_start, best_cost_from_parent, best_path_from_parent = self._find_best_parent(sample)
        if best_parent is None:
            return
        new_node = RRTNode(sample, parent=best_parent,
                           cost_from_start=best_cost_from_start,
                           cost_from_parent=best_cost_from_parent,
                           path_from_parent=best_path_from_parent)
        best_parent.children.append(new_node)

        # Reroute nodes in the neighboring area
        self._reroute(new_node)
        self.nodes.add(new_node)

        # Check if the new node reaches the goal directly
        self._set_goal_link(new_node)

    def _sample_new_pose(self) -> Pose_t:
        while True:
            current_sample = self._uniform_sample()
            current_idx = tuple(self.occ_map.pose_to_index(current_sample))
            if not self.occ_map.has_collision(current_idx):
                return current_sample

    def _uniform_sample(self) -> Pose_t:
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

    def _reroute(self, new_node: RRTNode) -> None:
        for neighbor in self.nodes:
            euclidean_distance = (neighbor.pose[0] - new_node.pose[0]) ** 2 + (neighbor.pose[1] - new_node.pose[1]) ** 2
            if euclidean_distance <= self.neighboring_range_square:
                path_sample_to_neighbor = get_reeds_shepp_path(new_node.pose, neighbor.pose, self.radius)
                cost_from_start_via_sample = new_node.cost_from_start + path_sample_to_neighbor.distance
                if cost_from_start_via_sample <= neighbor.cost_from_start and \
                    self._is_path_passable(new_node.pose, path_sample_to_neighbor):
                    neighbor.parent.children.remove(neighbor)
                    neighbor.parent = new_node
                    neighbor.path_from_parent = path_sample_to_neighbor
                    new_node.children.append(neighbor)

    def _find_best_parent(self, sample: Pose_t) -> tuple[RRTNode | None, float, float, ReedsSheppPath | None]:
        best_parent: RRTNode or None = None
        best_cost_from_start: float = np.inf
        best_cost_from_parent: float = np.inf
        best_path_from_parent: ReedsSheppPath or None = None
        for candidate in self.nodes:
            # Check whether it's reachable from the neighbor
            path_neighbor_to_sample = get_reeds_shepp_path(candidate.pose, sample, self.radius)
            new_cost_from_start = path_neighbor_to_sample.distance + candidate.cost_from_start
            if new_cost_from_start >= best_cost_from_start:
                continue

            if self._is_path_passable(candidate.pose, path_neighbor_to_sample):
                best_parent = candidate
                best_cost_from_start = new_cost_from_start
                best_cost_from_parent = path_neighbor_to_sample.distance
                best_path_from_parent = path_neighbor_to_sample

        return best_parent, best_cost_from_start, best_cost_from_parent, best_path_from_parent

    @staticmethod
    def _update_parents_cost_to_end(child_node: RRTNode) -> None:
        cur_node: RRTNode = child_node
        while cur_node.parent is not None:
            cost = cur_node.cost_to_goal + cur_node.cost_from_parent
            cur_node = cur_node.parent
            cur_node.cost_to_goal = cost

    def _update_children_cost_from_start(self, parent_node: RRTNode) -> None:
        cur_node: RRTNode = parent_node
        for child in cur_node.children:
            child.cost_from_start = parent_node.cost_from_start + child.cost_from_parent
            self._update_children_cost_from_start(child)

    def _set_goal_link(self, sample_node: RRTNode):
        path_sample_to_goal = get_reeds_shepp_path(sample_node.pose, self.goal, self.radius)
        if self._is_path_passable(sample_node.pose, path_sample_to_goal):
            new_cost_start_to_goal = path_sample_to_goal.distance + sample_node.cost_from_start
            if new_cost_start_to_goal < self.goal_node.cost_from_start:
                self.goal_reached = True
                sample_node.children.append(self.goal_node)
                sample_node.cost_to_goal = path_sample_to_goal.distance
                self.nodes.add(self.goal_node)
                if self.goal_node.parent is not None:
                    self.goal_node.parent.children.remove(self.goal_node)
                self.goal_node.parent = sample_node
                self.goal_node.cost_from_start = new_cost_start_to_goal
                self.goal_node.cost_from_parent = path_sample_to_goal.distance
                self.goal_node.path_from_parent = path_sample_to_goal
                self._update_parents_cost_to_end(self.goal_node)
