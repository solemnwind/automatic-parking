import numpy as np
import matplotlib.pyplot as plt
from models.environment import Environment
from algorithms.astar_search import AstarSearch, AstarNode


class Simulator:
    def __init__(self, config: str):
        self.env = Environment(config)
        self.car = self.env.car
        planner_params_ = {"env": self.env,
                           "start": (2.3, 3.0, np.pi / 2),
                           "goal": (0.7, 5.2, np.pi / 2),
                           "angle_resolution": 120,
                           "reverse_penalty": 1.5,
                           "error_goal_meter": 0.2,
                           "error_goal_radian": np.pi / 120}

        astar_params_ = {"heuristic_weight": 2.5,
                         "use_reeds_shepp": True,
                         "step_length": 0.2}

        vehicle_params_ = {"minimum_turning_radius": self.car.minimum_turning_radius,
                           "maximum_steering_angle": self.car.max_steering_angle,
                           "wheelbase": self.car.wheelbase}
        self.search = AstarSearch(planner_params_, astar_params_, vehicle_params_).search

    def run(self):
        path = self.search()
        if path is not None:
            fig, ax = self.env.draw()

            plt.plot([p[0] for p in path], [p[1] for p in path], 'r--')

            for pose in path[::2]:
                self.car.update(pose)
                self.car.draw(fig, ax)

            plt.show()


if __name__ == '__main__':
    from pathlib import Path
    file = Path(__file__).resolve()
    config_file = file.parent.parent / 'utils/test_parking_lot.toml'
    sim = Simulator(str(config_file))

    sim.run()
