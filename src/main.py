from pathlib import Path
import toml
import logging
from simulations.simulator import Simulator


def main():
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("[" + __name__ + "]")

    toml_file = str(Path(__file__).resolve().parent / 'utils/test_parking_lot_1.toml')

    with open(toml_file, 'r') as f:
        configuration = toml.loads(f.read())
        logger.info('Read scene config: %s', toml_file)

        Simulator(configuration).run()


if __name__ == '__main__':
    main()
