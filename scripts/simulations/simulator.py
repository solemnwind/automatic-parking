import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from matplotlib.lines import Line2D
import matplotlib.animation as animation
from models.environment import Environment
from models.car import Car
from models.utils import Pose_t, mod2pi
from algorithms.astar_search import AStarSearch
from algorithms.rrtstar import RRTStar


class Simulator:
    def __init__(self, config: dict):
        self.config = config
        config['scene']['obstacles'] = [tuple(obs) for obs in config['scene']['obstacles']]

        self.env = Environment(config["scene"])
        self.car = Car(config["vehicle"])

        start_pose = config["planner"]["start_pose"]
        goal_pose = config["planner"]["goal_pose"]
        start_pose[2] = mod2pi(start_pose[2] * np.pi / 180)
        goal_pose[2] = mod2pi(goal_pose[2] * np.pi / 180)

        self.car.update(start_pose)

        planner_params_ = {"env": self.env,
                           "car": self.car,
                           "start": start_pose,
                           "goal": goal_pose,
                           "angle_resolution": int(config["planner"]["angle_resolution"]),
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

        self.search = RRTStar(planner_params_, astar_params_, vehicle_params_).search

    def run(self):
        path = self.search()
        if path is not None:
            if self.config["result"]["save_gif"]:
                self._draw_gif(path, False)

            self._draw_png(path, True)

    def _draw_references(self, ax: plt.Axes, **kwargs):
        # Highlight the start and goal pose in red and green colors
        self.car.update(self.config["planner"]["start_pose"])
        ax.add_patch(self.car.patch(redraw=True, color="red"))
        self.car.update(self.config["planner"]["goal_pose"])
        ax.add_patch(self.car.patch(redraw=True, color="green"))

        red_patch = Patch(edgecolor='red', fill=False, label='Start Pose')
        green_patch = Patch(edgecolor='green', fill=False, label='Goal Pose')
        blue_patch = Patch(edgecolor='blue', fill=False, label='Vehicle Pose')
        red_line = Line2D([0], [0], color='red', linewidth=1, linestyle='--', label='Path')
        yellow_patch = Patch(edgecolor='gold', linestyle='--', fill=False, label='Parking Spot')
        black_patch = Patch(facecolor='black', alpha=0.7, edgecolor=None, label='Obstacles')
        handles = [red_patch, green_patch, blue_patch, red_line, yellow_patch, black_patch]

        plt.legend(handles=handles, **kwargs)

    def _draw_png(self, path: list[Pose_t], show=True):
        fig, ax = self.env.draw()

        plt.plot([p[0] for p in path], [p[1] for p in path], 'r--')

        for pose in path:
            self.car.update(pose)
            ax.add_patch(self.car.patch(redraw=True))

        self._draw_references(ax, loc='center left', bbox_to_anchor=(1, 0.5))

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

        self._draw_references(ax, loc='center left', bbox_to_anchor=(1, 0.5))

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
