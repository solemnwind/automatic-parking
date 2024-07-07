from heapq import heappush, heappop
import numpy as np
from models.environment import Environment
from models.reeds_shepp import ReedsShepp

if __name__ == '__main__':
    from pathlib import Path  # if you haven't already done so
    file = Path(__file__).resolve()
    config_file = file.parent.parent / 'utils/test_parking_lot.toml'
    env = Environment(config_file)

    planner_params = {"env": env,
                      "start": (3, 2, 90),
                      "goal": (0.5, 3, 270)}

    astar_params = {"heuristic_weight": 1.0,
                    "use_reeds_shepp": True}

    vehicle_params = {"minimum_turning_radius": env.car.minimum_turning_radius}

class AstarSearch:
    def __init__(self, planner_params: dict, astar_params: dict, vehicle_params: dict):
        self.env = planner_params["env"]
        self.start = planner_params["start"]
        self.goal = planner_params["goal"]
        self.radius = vehicle_params["minimum_turning_radius"]
        self.astar_params = astar_params

    def search(self):
        start_idx = tuple(self.env.metric_to_index(self.start))
        goal_idx = tuple(self.env.metric_to_index(self.goal))

    def _heuristic_cost(self, pose) -> float:
        if self.astar_params["use_reeds_shepp"]:
            return ReedsShepp(pose, self.goal, self.radius).distance * self.astar_params["heuristic_weight"]
        else:
            return np.linalg.norm(self.goal[:2] - pose[:2]) * self.astar_params["heuristic_weight"]
