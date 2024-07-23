project(_reeds_shepp)

pybind11_add_module(_reeds_shepp reeds_shepp.h reeds_shepp.cpp)

set(DEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/models)

install(TARGETS _reeds_shepp
	LIBRARY DESTINATION ${DEST_DIR})