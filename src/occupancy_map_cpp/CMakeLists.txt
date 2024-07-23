project(_occupancy_map)

pybind11_add_module(_occupancy_map occupancy_map.h occupancy_map.cpp)

set(DEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/models)

install(TARGETS _occupancy_map
	LIBRARY DESTINATION ${DEST_DIR})