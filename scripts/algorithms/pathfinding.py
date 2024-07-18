import logging
from models import OccupancyMap
from models.utils import Pose_t
from models.environment import Environment
from models.car import Car

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")

class PathFindingAlgorithm:
    def __init__(self, planner_params: dict, vehicle_params: dict):
        self.planner_params = planner_params
        self.vehicle_params = vehicle_params
        self.start: Pose_t = planner_params["start"]
        self.goal: Pose_t = planner_params["goal"]
        self.radius: float = vehicle_params["minimum_turning_radius"]
        self.angle_resolution: int = planner_params["angle_resolution"]

        self.env: Environment = planner_params["env"]
        self.car: Car = planner_params["car"]

        logger.info("Initializing occupancy map...")
        self.occ_map = OccupancyMap((self.env.west, self.env.east, self.env.south, self.env.north),
                                    self.env.obstacles, self.env.resolution, self.angle_resolution,
                                    self.car.front_to_base_axle, self.car.rear_to_base_axle, self.car.width)

    def near_goal(self, pose: Pose_t):
        if abs(pose[0] - self.goal[0]) < self.planner_params["error_goal_meter"] and \
                abs(pose[1] - self.goal[1]) < self.planner_params["error_goal_meter"] and \
                abs(pose[2] - self.goal[2]) < self.planner_params["error_goal_radian"]:
            return True
        return False
