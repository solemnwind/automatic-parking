import toml
import numpy as np
import matplotlib.pyplot as plt
import logging
from models.environment import Environment
from models.car import Car
from algorithms.astar_search import AstarSearch

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Simulator:
    def __init__(self, toml_file: str):
        with open(toml_file, 'r') as f:
            config = toml.loads(f.read())
            logger.info('Read scene config: %s', toml_file)

            self.env = Environment(config["scene"])
            self.car = Car(config["vehicle"])

            start_pose = config["planner"]["start_pose"]
            goal_pose = config["planner"]["goal_pose"]
            start_pose[2] *= np.pi / 180
            goal_pose[2] *= np.pi / 180

            self.car.update(start_pose)

            planner_params_ = {"env": self.env,
                               "car": self.car,
                               "start": start_pose,
                               "goal": goal_pose,
                               "angle_resolution": config["planner"]["angle_resolution"],
                               "reverse_penalty": config["planner"]["reverse_penalty"],
                               "error_goal_meter": config["planner"]["error_goal_meter"],
                               "error_goal_radian": config["planner"]["error_goal_degree"] * np.pi / 180
                               }

            astar_params_ = {"heuristic_weight": config["astar"]["heuristic_weight"],
                             "step_length": config["astar"]["step_length"]
                             }

            vehicle_params_ = {"minimum_turning_radius": self.car.minimum_turning_radius,
                               "maximum_steering_angle": self.car.max_steering_angle,
                               "wheelbase": self.car.wheelbase
                               }

            self.search = AstarSearch(planner_params_, astar_params_, vehicle_params_).search

    def run(self):
        path = self.search()
        if path is not None:
            fig, ax = self.env.draw()

            plt.plot([p[0] for p in path], [p[1] for p in path], 'r--')

            for pose in path:
                self.car.update(pose)
                self.car.draw(fig, ax)

            plt.show()


if __name__ == '__main__':
    from pathlib import Path
    file = Path(__file__).resolve()
    config = str(file.parent.parent / 'utils/test_parking_lot.toml')

    Simulator(config).run()
