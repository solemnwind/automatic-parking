#include "reeds_shepp.h"

namespace py = pybind11;
using namespace std;


double mod2pi(double theta) {
    double phi = fmod(theta, 2 * M_PI);
    if (phi > M_PI) {
        phi -= M_PI;
    } else if (phi <= -M_PI) {
        phi += M_PI;
    }
    return phi;
}

void polar(double x, double y, double &r, double &phi) {
    r = sqrt(x * x + y * y);
    phi = atan2(y, x);
}

ReedsShepp::ReedsShepp(Pose_t start, Pose_t goal, double radius)
    : m_start(start), m_goal(goal), m_radius(radius)
{
    Pose_t normalizedGoal = transformToOrigin(m_start, m_goal);
    m_x = normalizedGoal[0];
    m_y = normalizedGoal[1];
    m_phi = normalizedGoal[2];
}

Pose_t ReedsShepp::transformToOrigin(Pose_t &start, Pose_t &goal) {
    double dx = goal[0] - start[0];
    double dy = goal[1] - start[1];
    double theta = -start[2];

    return Pose_t{dx * cos(theta) - dy * sin(theta),
                  dx * sin(theta) + dy * cos(theta),
                  mod2pi(goal[2] - start[2])};
}

bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v) {
    polar(x - sin(phi), y - 1.0 + cos(phi), u, t);
    if (t >= -EPS) {
        v = mod2pi(phi - t);
        if (v >= -EPS) {
            return true;
        }
    }
    return false;
}

bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v) {
    return false;
}


double getReedsSheppDistance(double x_start, double y_start, double phi_start,
                             double x_goal, double y_goal, double phi_goal,
                             double radius) {
    ReedsShepp rs({x_start, y_start, phi_start},
                  {x_goal, y_goal, phi_goal}, radius);
    return 1.0;
}


PYBIND11_MODULE(reeds_shepp, handle) {
    handle.doc() = "Get the shortest Reeds-Shepp distance.";

    handle.def("reeds_shepp", &getReedsSheppDistance,
               "Get the shortest Reeds-Shepp distance.");
}
