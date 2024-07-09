from heapq import heappush, heappop, heapify
import numpy as np
from models.environment import Environment
from models.reeds_shepp import ReedsShepp
from typing import Self
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")

Pose_t = tuple[float, float, float]
Idx_t = tuple[int, int, int]


def getNewPose(pose: Pose_t, delta: Pose_t) -> Pose_t:
    return pose[0] + delta[0], pose[1] + delta[1], pose[2] + delta[2]


class AstarNode:
    def __init__(self, **kwargs):
        self.open_status: bool or None = None  # True: open; False: closed; None: new node
        self.pose: Pose_t or None = None
        self.g_score: float = 0
        self.h_score: float = np.inf
        self.parent: Self or None = None
        if kwargs:
            self.set(kwargs)

    def set(self,
            open_status: bool or None = None,
            pose: Pose_t or None = None,
            g_score: float or None = None,
            h_score: float or None = None,
            parent: Self or None = None):
        if open_status is not None:
            self.open_status = open_status
        if pose is not None:
            self.pose = pose
        if g_score is not None:
            self.g_score = g_score
        if h_score is not None:
            self.h_score = h_score
        if parent is not None:
            self.parent = parent

    def get_path(self) -> list[Pose_t]:
        if self.parent is None:
            return [self.pose]
        return self.parent.get_path() + self.pose


    def cost(self):
        return self.g_score + self.h_score

    def __lt__(self, other):
        return self.cost() < other.cost()

    def __gt__(self, other):
        return self.cost() > other.cost()


class AstarSearch:
    def __init__(self, planner_params: dict, astar_params: dict, vehicle_params: dict):
        self.env: Environment = planner_params["env"]
        self.start: Pose_t = planner_params["start"]
        self.goal: Pose_t = planner_params["goal"]
        self.radius: float = vehicle_params["minimum_turning_radius"]
        self.astar_params = astar_params

    def search(self) -> list[AstarNode] or None:
        # Initialize openlist with start node
        openlist: list[AstarNode] = []  # maintains a priority queue of (the references of) AstarNodes
        discovered_map: dict[Idx_t, AstarNode] = {}  # a unordered map storing discovered AstarNodes

        start_idx = self.env.metric_to_index(self.start)
        discovered_map[start_idx] = AstarNode(open_status=True,
                                              pose=self.start,
                                              g_score=0,
                                              h_score=self._heuristic_cost(self.start))
        heappush(openlist, discovered_map[start_idx])

        while openlist:
            cur_node = heappop(openlist)
            if cur_node.open_status is True:
                cur_node.set(open_status=False)
            else:
                continue

            if self._near_goal(cur_node.pose):
                logger.info("A path is found.")
                return cur_node.get_path()

            # TODO: define Transition class and transition_table
            transitions = []
            for transition in transitions:
                # TODO: different weight for changing gear

                next_pose = getNewPose(cur_node.pose, transition.delta)
                # TODO: next_idx = ?
                next_idx = self.env.metric_to_index(next_pose)

                next_g_cost = transition.distance + cur_node.g_score
                next_h_cost = self._heuristic_cost(next_pose)

                if self.env.has_collision(next_idx): # TODO: collision detection
                    continue

                if next_idx not in discovered_map:
                    discovered_map[next_idx] = AstarNode(open_status=True,
                                                         pose=next_pose,
                                                         g_score=next_g_cost,
                                                         h_score=next_h_cost,
                                                         parent=cur_node)
                    heappush(openlist, discovered_map[next_idx])
                else:
                    next_node = discovered_map[next_idx]
                    if next_node.open_status is True and next_g_cost < next_node.g_score:
                        next_node.set(g_score=next_g_cost,
                                      h_score=next_h_cost,
                                      parent=cur_node)
                        heappush(openlist, next_node)

        logger.warning("No path is found!")
        return None

    def _heuristic_cost(self, pose) -> float:
        if self.astar_params["use_reeds_shepp"]:
            return ReedsShepp(pose, self.goal, self.radius).distance * self.astar_params["heuristic_weight"]
        else:
            return np.linalg.norm(np.array(self.goal[:2]) - np.array(pose[:2])) * self.astar_params["heuristic_weight"]

    def _near_goal(self, pose):
        # TODO: return True if pose is near the goal
        if pose == self.goal:
            return True
        return False

    def _get_path(self, pose):
        return []


if __name__ == '__main__':
    from pathlib import Path

    file = Path(__file__).resolve()
    config_file = file.parent.parent / 'utils/test_parking_lot.toml'
    env = Environment(config_file)

    planner_params_ = {"env": env,
                       "start": (3, 2, 90),
                       "goal": (0.5, 3, 270)}

    astar_params_ = {"heuristic_weight": 1.0,
                     "use_reeds_shepp": True}

    vehicle_params_ = {"minimum_turning_radius": env.car.minimum_turning_radius}
