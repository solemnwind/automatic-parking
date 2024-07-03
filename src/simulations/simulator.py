from models.environment import Environment


class Simulator:
    def __init__(self):
        pass


if __name__ == '__main__':
    # config_file = '../utils/test_parking_lot.toml'
    # env = Environment(config_file)
    import matplotlib.pyplot as plt
    import numpy as np

    # import ctypes
    #
    # dll = ctypes.CDLL("../../reeds-shepp-cppbind/reeds-shepp/out/build/x64-release/reeds_shepp.dll")


    # def plot_dubins_path(start, end, turning_radius):
    #     path = dubins.shortest_path(start, end, turning_radius)
    #     configurations, _ = path.sample_many(0.1)  # Sample points along the path
    #
    #     x = [config[0] for config in configurations]
    #     y = [config[1] for config in configurations]
    #     theta = [config[2] for config in configurations]
    #
    #     plt.plot(x, y, label="Dubins Path")
    #     plt.scatter([start[0], end[0]], [start[1], end[1]], c='red')  # Start and end points
    #     plt.arrow(start[0], start[1], 0.5 * np.cos(start[2]), 0.5 * np.sin(start[2]), head_width=0.1, color='red')
    #     plt.arrow(end[0], end[1], 0.5 * np.cos(end[2]), 0.5 * np.sin(end[2]), head_width=0.1, color='red')
    #     plt.axis("equal")
    #     plt.legend()
    #     plt.show()
    #
    #
    # start = (0, 0, np.deg2rad(0))
    # end = (10, 10, np.deg2rad(90))
    # turning_radius = 1.0
    #
    # plot_dubins_path(start, end, turning_radius)
