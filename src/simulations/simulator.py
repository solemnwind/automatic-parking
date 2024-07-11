import toml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import logging
from models.environment import Environment
from models.car import Car
from models.utils import Pose_t
from algorithms.astar_search import AstarSearch

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")


class Simulator:
    def __init__(self, config: dict):
        self.config = config
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
            if self.config["result"]["save_gif"]:
                self._draw_gif(path, False)
            if self.config["result"]["save_png"]:
                self._draw_png(path, True)

    def _draw_png(self, path: list[Pose_t], show=True):
        fig, ax = self.env.draw()

        plt.plot([p[0] for p in path], [p[1] for p in path], 'r--')
        for pose in path:
            self.car.update(pose)
            ax.add_patch(self.car.patch(redraw=True))

        if self.config["result"]["save_png"]:
            plt.savefig(self.config["result"]["png_path"])

        if show:
            plt.show()

    def _draw_gif(self, path: list[Pose_t], show=True):
        fig, ax = self.env.draw()

        start_freeze = 20
        goal_freeze = 40
        total_frames = len(path) + start_freeze + goal_freeze - 1

        x = [p[0] for p in path]
        y = [p[1] for p in path]

        def animate(i):
            line = ax.plot([], [], 'r--')
            if i < start_freeze:
                self.car.update(path[0])
            elif i < len(path) + start_freeze:
                self.car.update(path[i - start_freeze])
                line = ax.plot(x[:i - start_freeze], y[:i - start_freeze], 'r--')
            return ax.add_patch(self.car.patch(redraw=False)), line

        ani = animation.FuncAnimation(fig, animate, repeat=True,
                                      frames=total_frames, interval=50)
        ani.save(self.config["result"]["gif_path"], writer='imagemagick', fps=20)

        if show:
            plt.show()


if __name__ == '__main__':
    from pathlib import Path
    toml_file = str(Path(__file__).resolve().parent.parent / 'utils/test_parking_lot.toml')

    with open(toml_file, 'r') as f:
        configuration = toml.loads(f.read())
        logger.info('Read scene config: %s', toml_file)

        Simulator(configuration).run()
