#ifndef OCCUPANCY_MAP_H
#define OCCUPANCY_MAP_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <array>
#include <utility>
#include <map>

namespace py = pybind11;

#define EPS 1e-6

typedef std::array<double, 2> coord_t;
typedef std::array<int, 2> index_t;
typedef std::array<double, 4> bound_t;

class OccupancyMap
{
private:
    double m_resolution;
    int m_angleResolution;
    double m_unitAngle;
    coord_t m_origin;
    int m_width, m_height;
    std::vector<std::vector<bool>> m_map;
    std::vector<std::vector<index_t>> m_collisionLUTs;

public:
    OccupancyMap(const bound_t &bounds, const std::vector<bound_t> &obstacles,
                 double resolution, int angleResolution,
                 double vehicleFrontToBase, double vehicleRearToBase, double vehicleWidth);

    [[nodiscard]]
    std::array<int, 3> poseToIndex(std::array<double, 3> pose) const;

    [[nodiscard]]
    bool hasCollision(std::array<int, 3> poseIndices) const;

private:
    [[nodiscard]]
    index_t convertMetricToIndexFloor(coord_t metrics) const
    {
        return index_t{(int)floor((metrics[0] - m_origin[0]) / m_resolution),
                       (int)floor((metrics[1] - m_origin[1]) / m_resolution)};
    };

    [[nodiscard]]
    index_t convertMetricToIndexCeil(coord_t metrics) const
    {
        return index_t{(int)ceil((metrics[0] - m_origin[0]) / m_resolution),
                       (int)ceil((metrics[1] - m_origin[1]) / m_resolution)};
    };

    [[nodiscard]]
    bool isOutOfBound(int u, int v) const
    {
        return (u < 0 || v < 0 || u >= m_width || v >= m_height);
    }

    void createCollisionLUT(double vehicleFrontToBase, double vehicleRearToBase, double vehicleWidth);
};


#endif //OCCUPANCY_MAP_H
