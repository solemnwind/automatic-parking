from heapq import heappush, heappop
import numpy as np
from typing import Self
import logging
from models.utils import Pose_t, Idx_t, addPose, discretize_angle, recover_angle

use_cpp_modules = True
if use_cpp_modules:
    from models._reeds_shepp import get_distance
    from models._occupancy_map import OccupancyMap
else:
    from models.reeds_shepp import get_distance
    from models.occupancy_map import OccupancyMap

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Transition:
    def __init__(self, delta: Pose_t, is_back: bool, distance: float):
        self.delta: Pose_t = delta
        self.reverse: bool = is_back
        self.distance: float = distance


def create_transition_lut(planner_params, astar_params, vehicle_params) -> list[list[Transition]]:
    # Read parameters
    theta = vehicle_params["maximum_steering_angle"]
    wheelbase = vehicle_params["wheelbase"]
    angle_resolution = planner_params["angle_resolution"]
    step = astar_params["step_length"]
    # Pre-compute all the turning radius of possible steering angles
    maximum_steering_angle_index = discretize_angle(theta, angle_resolution)
    possible_steering_angle_indices = list(range(1, maximum_steering_angle_index + 1))
    turns: dict[int, tuple[float, float]] = {}  # key: index, value: (dphi, steering radius)
    for idx in possible_steering_angle_indices:
        steering_angle = recover_angle(idx, angle_resolution)
        r = wheelbase / np.tan(abs(steering_angle))
        turns[idx] = (step / r, r)

    lut = []
    for i in range(angle_resolution):
        phi = recover_angle(i, angle_resolution)
        candidate_transitions = []
        # Straight forward/backward
        dx = step * np.cos(phi)
        dy = step * np.sin(phi)
        candidate_transitions.append(Transition((dx, dy, 0), False, step))
        candidate_transitions.append(Transition((-dx, -dy, 0), True, step))

        # Turning
        for idx in range(1, maximum_steering_angle_index + 1):
            dphi, r = turns[idx]
            dx_ = r * np.sin(dphi)
            dy_ = r * (1 - np.cos(dphi))
            dx = dx_ * np.cos(phi) - dy_ * np.sin(phi)
            dy = dx_ * np.sin(phi) + dy_ * np.cos(phi)
            candidate_transitions.append(Transition((dx, dy, dphi), False, step))
            candidate_transitions.append(Transition((-dx, -dy, dphi), True, step))
            dx = dx_ * np.cos(phi) + dy_ * np.sin(phi)
            dy = dx_ * np.sin(phi) - dy_ * np.cos(phi)
            candidate_transitions.append(Transition((dx, dy, -dphi), False, step))
            candidate_transitions.append(Transition((-dx, -dy, -dphi), True, step))

        lut.append(candidate_transitions)

    return lut


class AStarNode:
    def __init__(self, **kwargs):
        self.open: bool or None = None  # True: open; False: closed; None: new node
        self.pose: Pose_t or None = None
        self.g_score: float = 0
        self.h_score: float = np.inf
        self.parent: Self or None = None
        self.reverse: bool = False
        if kwargs:
            self.set(**kwargs)

    def set(self,
            open: bool or None = None,
            pose: Pose_t or None = None,
            g_score: float or None = None,
            h_score: float or None = None,
            parent: Self or None = None,
            reverse: bool or None = None):
        if open is not None:
            self.open = open
        if pose is not None:
            self.pose = pose
        if g_score is not None:
            self.g_score = g_score
        if h_score is not None:
            self.h_score = h_score
        if parent is not None:
            self.parent = parent
        if reverse is not None:
            self.reverse = reverse

    def get_path(self) -> list[Pose_t]:
        if self.parent is None:
            return [self.pose]
        return self.parent.get_path() + [self.pose]

    def cost(self):
        return self.g_score + self.h_score

    def __lt__(self, other):
        return self.cost() < other.cost()


class AStarSearch:
    def __init__(self, planner_params: dict, astar_params: dict, vehicle_params: dict):
        self.planner_params = planner_params
        self.astar_params = astar_params
        self.vehicle_params = vehicle_params
        self.start: Pose_t = planner_params["start"]
        self.goal: Pose_t = planner_params["goal"]
        self.radius: float = vehicle_params["minimum_turning_radius"]
        self.angle_resolution: int = planner_params["angle_resolution"]
        self.transition_lut = create_transition_lut(planner_params, astar_params, vehicle_params)

        env = planner_params["env"]
        car = planner_params["car"]
        self.occ_map = OccupancyMap((env.west, env.east, env.south, env.north), env.obstacles, env.resolution,
                                    self.angle_resolution, car.front_to_base_axle, car.rear_to_base_axle, car.width)

    def search(self) -> list[AStarNode] or None:
        # Initialize openlist with start node
        openlist: list[AStarNode] = []  # maintains a priority queue of (the references of) AstarNodes
        discovered_map: dict[Idx_t, AStarNode] = {}  # a unordered map storing discovered AstarNodes

        start_idx = tuple(self.occ_map.pose_to_index(self.start))
        discovered_map[start_idx] = AStarNode(open=True,
                                              pose=self.start,
                                              g_score=0,
                                              h_score=self._heuristic_cost(self.start))
        heappush(openlist, discovered_map[start_idx])

        while openlist:
            cur_node = heappop(openlist)
            logger.debug(cur_node.pose)
            if not cur_node.open:
                continue

            cur_node.set(open=False)

            if self._near_goal(cur_node.pose):
                logger.info("A path is found.")
                return cur_node.get_path()

            # Iterate neighbors
            transitions = self.transition_lut[discretize_angle(cur_node.pose[2], self.angle_resolution)]
            for transition in transitions:
                next_pose = addPose(cur_node.pose, transition.delta)
                next_idx = tuple(self.occ_map.pose_to_index(next_pose))
                if self.occ_map.has_collision(next_idx):
                    continue

                next_g_cost = transition.distance + cur_node.g_score
                if cur_node.reverse != transition.reverse:
                    next_g_cost *= self.planner_params["reverse_penalty"]

                if next_idx not in discovered_map:
                    discovered_map[next_idx] = AStarNode(open=True,
                                                         pose=next_pose,
                                                         g_score=next_g_cost,
                                                         h_score=self._heuristic_cost(next_pose),
                                                         parent=cur_node,
                                                         reverse=transition.reverse)
                    heappush(openlist, discovered_map[next_idx])
                else:
                    next_node = discovered_map[next_idx]
                    if next_node.open is True and next_g_cost < next_node.g_score:
                        next_node.set(g_score=next_g_cost,
                                      parent=cur_node,
                                      reverse=transition.reverse)
                        heappush(openlist, next_node)

        logger.warning("No path is found!")
        return None

    def _heuristic_cost(self, pose) -> float:
        return get_distance(pose[0], pose[1], pose[2],
                            self.goal[0], self.goal[1], self.goal[2],
                            self.radius) * \
            self.astar_params["heuristic_weight"]

    def _near_goal(self, pose: Pose_t):
        # TODO: return True if pose is near the goal
        if abs(pose[0] - self.goal[0]) < self.planner_params["error_goal_meter"] and \
           abs(pose[1] - self.goal[1]) < self.planner_params["error_goal_meter"] and \
           abs(pose[2] - self.goal[2]) < self.planner_params["error_goal_radian"]:
            return True
        return False
