#include "occupancy_map.h"

OccupancyMap::OccupancyMap(const bound_t &bounds, const std::vector<bound_t> &obstacles,
                           double resolution, int angleResolution,
                           double vehicleFrontToBase, double vehicleRearToBase, double vehicleWidth):
        m_resolution(resolution),
        m_angleResolution(angleResolution),
        m_unitAngle(2. * M_PI / angleResolution),
        m_origin({bounds[0], bounds[2]}),
        m_width(ceil((bounds[1] - bounds[0]) / m_resolution) ),
        m_height(ceil((bounds[3] - bounds[2]) / m_resolution)),
        m_map(std::vector(m_width, std::vector<bool>(m_height, false))),
        m_collisionLUTs(std::vector(m_angleResolution, std::vector<index_t>()))
{
    for (const std::array<double, 4> &obstacle : obstacles) {
        index_t bottomLeft = convertMetricToIndexCeil({obstacle[0] - EPS, obstacle[2] - EPS});
        index_t topRight = convertMetricToIndexFloor({obstacle[1] + EPS, obstacle[3] + EPS});
        for (int u = bottomLeft[0] - 1; u <= topRight[0]; ++u) {
            for (int v = bottomLeft[1] - 1; v <= topRight[1]; ++v) {
                if (isOutOfBound(u, v)) { continue; }
                m_map[u][v] = true;
            }
        }
    }

    createCollisionLUT(vehicleFrontToBase, vehicleRearToBase, vehicleWidth);
}

void OccupancyMap::createCollisionLUT(double vehicleFrontToBase, double vehicleRearToBase, double vehicleWidth)
{
    double frontToBase = vehicleFrontToBase;
    double rearToBase = vehicleRearToBase;
    double halfWidth = vehicleWidth / 2.;
    double edgeSamples[] = {0.,  0.2, 0.3, 0.4,
                            0.5, 0.6, 0.7, 0.8};
    for (int i = 0; i < m_angleResolution; ++i) {
        double phi = i * m_unitAngle;
        // Add corners
        coord_t frontLeft{frontToBase * cos(phi) - halfWidth * sin(phi),
                          frontToBase * sin(phi) + halfWidth * cos(phi)};
        coord_t frontRight{frontToBase * cos(phi) + halfWidth * sin(phi),
                           frontToBase * sin(phi) - halfWidth * cos(phi)};
        coord_t rearLeft{-rearToBase * cos(phi) - halfWidth * sin(phi),
                         -rearToBase * sin(phi) + halfWidth * cos(phi)};
        coord_t rearRight{-rearToBase * cos(phi) + halfWidth * sin(phi),
                          -rearToBase * sin(phi) - halfWidth * cos(phi)};
        // Sample points on the edges
        std::array<std::pair<coord_t, coord_t>, 4> edges {
            std::pair{frontLeft, frontRight},
            std::pair{frontRight, rearRight},
            std::pair{rearRight, rearLeft},
            std::pair{rearLeft, frontLeft}
        };
        for (auto &edge : edges) {
            for (double t : edgeSamples) {
                coord_t sample{edge.first[0] * (1. - t) + edge.second[0] * t,
                               edge.first[1] * (1. - t) + edge.second[1] * t};
                m_collisionLUTs[i].push_back({(int)floor(sample[0] / m_resolution),
                                              (int)floor(sample[1] / m_resolution)});
            }
        }
        // Also check the base
        m_collisionLUTs[i].push_back({0, 0});
    }
}

std::array<int, 3> OccupancyMap::poseToIndex(std::array<double, 3> pose) const
{
    int angleIndex = int(fmod(pose[2] + m_unitAngle / 2., 2. * M_PI) / m_unitAngle);
    angleIndex = angleIndex < m_angleResolution ? angleIndex : angleIndex - m_angleResolution;
    angleIndex = angleIndex >= 0 ? angleIndex : angleIndex + m_angleResolution;
    index_t xyIndex = convertMetricToIndexFloor({pose[0], pose[1]});
    return std::array<int, 3>{xyIndex[0], xyIndex[1], angleIndex};
}

bool OccupancyMap::hasCollision(std::array<int, 3> poseIndices) const
{
    for (const index_t &localIndices : m_collisionLUTs[poseIndices[2]]) {
        int u = localIndices[0] + poseIndices[0];
        int v = localIndices[1] + poseIndices[1];
        if (isOutOfBound(u, v)) { return true; }
        if (m_map[u][v]) { return true; }
    }
    return false;
}


PYBIND11_MODULE(_occupancy_map, m)
{
    py::class_<OccupancyMap>(m, "OccupancyMap")
        .def(py::init<const bound_t&, const std::vector<bound_t>, double, int, double, double, double>())
        .def("pose_to_index", &OccupancyMap::poseToIndex)
        .def("has_collision", &OccupancyMap::hasCollision);
}
