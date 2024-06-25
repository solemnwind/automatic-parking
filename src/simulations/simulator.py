from models.environment import Environment


class Simulator:
    def __init__(self):
        pass


if __name__ == '__main__':
    config_file = '../utils/test_parking_lot.toml'
    env = Environment(config_file)
