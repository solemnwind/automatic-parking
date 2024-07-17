import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("[" + __name__ + "]")

cpp_import_success_string = "Using C++ library: \033[1m{}\033[0m"
cpp_import_failure_string = "C++ library _reeds_shepp not found! Using fallback Python module: \033[1m{}\033[0m"

try:
    from ._reeds_shepp import get_reeds_shepp_distance, get_reeds_shepp_path, interpolate_reeds_shepp_path, ReedsSheppPath
    logger.info(cpp_import_success_string.format("_reeds_shepp"))
except ImportError:
    from .reeds_shepp import get_reeds_shepp_distance, get_reeds_shepp_path, interpolate_reeds_shepp_path, ReedsSheppPath
    logger.warning(cpp_import_failure_string.format("reeds_shepp"))

try:
    from ._occupancy_map import OccupancyMap
    logger.info(cpp_import_success_string.format("_occupancy_map"))
except ImportError:
    from .occupancy_map import OccupancyMap
    logger.warning(cpp_import_failure_string.format("occupancy_map"))
