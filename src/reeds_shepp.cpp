#include "reeds_shepp.h"

namespace py = pybind11;
using namespace std;


double mod2pi(double theta)
{
    double phi = fmod(theta, 2 * M_PI);
    if (phi > M_PI) {
        phi -= M_PI;
    } else if (phi <= -M_PI) {
        phi += M_PI;
    }
    return phi;
}

void polar(double x, double y, double &r, double &phi)
{
    r = sqrt(x * x + y * y);
    phi = atan2(y, x);
}

void tw(double u, double u1, double xi, double eta, double phi, double &tau, double &omega)
{
    double delta, A, B, t1, t2;
    delta = mod2pi(u - u1);
    A = sin(u) - sin(delta);
    B = cos(u) - cos(delta) - 1.;
    t1 = atan2(eta * A - xi * B, xi * A + eta * B);
    t2 = cos(delta) - cos(u1) - cos(u) + 1.5;
    tau = t2 < 0 ? mod2pi(t1 + M_PI) : mod2pi(t1);
    omega = mod2pi(tau - u + u1 - phi);
}

//====================== class ReedsShepp ======================
ReedsShepp::ReedsShepp(Pose_t start, Pose_t goal, double radius)
    : m_start(start), m_goal(goal), m_radius(radius)
{
    Pose_t normalizedGoal = transformToOrigin(m_start, m_goal);
    m_x = normalizedGoal[0];
    m_y = normalizedGoal[1];
    m_phi = normalizedGoal[2];

    CSC(path, )
}

Pose_t ReedsShepp::transformToOrigin(Pose_t &start, Pose_t &goal)
{
    double dx = goal[0] - start[0];
    double dy = goal[1] - start[1];
    double theta = -start[2];

    return Pose_t{dx * cos(theta) - dy * sin(theta),
                  dx * sin(theta) + dy * cos(theta),
                  mod2pi(goal[2] - start[2])};
}
//---------------------- class ReedsShepp ----------------------

//====================== class ReedsSheppPath ======================

//ReedsSheppPath::ReedsSheppPath(RSPathType pt)
//{
//    ;
//}

//---------------------- class ReedsSheppPath ----------------------

bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
{
    polar(x - sin(phi), y - 1. + cos(phi), u, t);
    if (t >= -EPS) {
        v = mod2pi(phi - t);
        if (v >= -EPS) {
            return true;
        }
    }
    return false;
}

bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
{
    double u1, t1;
    polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
    if (u1 * u1 >= 4.) {
        u = sqrt(u1 * u1 - 4.);
        double theta = atan2(2., u);
        t = mod2pi(t1 + theta);
        v = mod2pi(t - phi);
        return true;
    }
    return false;
}

bool LpRnL(double x, double y, double phi, double &t, double &u, double &v)
{
    double xi, eta, u1, theta;
    xi = x - sin(phi);
    eta = y - 1 + cos(phi);
    polar(xi, eta, u1, theta);
    if (u1 <= 4.) {
        u = 2. * asin(u1 / 4.);
        t = mod2pi(theta - u / 2. + M_PI);
        v = mod2pi(phi - u - t);
        if (t >= -EPS && u >= -EPS) {
            return true;
        }
    }
    return false;
}


bool LpRpuLnuRn(double x, double y, double phi, double &t, double &u, double &v)
{
    double xi, eta, rho;
    xi = x + sin(phi);
    eta = y - 1. - cos(phi);
    rho = (2. + sqrt(xi * xi + eta * eta)) / 4.;
    if (rho <= 1.) {
        u = acos(rho);
        tw(u, -u, xi, eta, phi, t, v);
        if (t >= -EPS && v <= EPS) {
            return true;
        }
    }
    return false;
}

bool LpRnuLnuRp(double x, double y, double phi, double &t, double &u, double &v)
{
    double xi, eta, rho;
    xi = x + sin(phi);
    eta = y - 1. - cos(phi);
    rho = (20. - xi * xi - eta * eta) / 16.;
    if (0 <= rho && rho <= 1) {
        u = -acos(rho);
        tw(u, u, xi, eta, phi, t, v);
        if (t >= -EPS && v >= -EPS) {
            return true;
        }
    }
    return false;
}

bool LpRnSnLn(double x, double y, double phi, double &t, double &u, double &v)
{
    double xi, eta, rho, theta;
    xi = x + sin(phi);
    eta = y - 1. - cos(phi);
    polar(-eta, xi, rho, theta);
    if (rho >= 2) {
        double theta1 = atan2(2, sqrt(rho * rho - 4.));
        t = mod2pi(theta - theta1);
        u = 2 - theta1;
        v = mod2pi(phi - M_PI_2 - t);
        if (t >= -EPS && u <= EPS && v <= EPS) {
            return true;
        }
    }
    return false;
}

bool LpRnSnRn(double x, double y, double phi, double &t, double &u, double &v)
{
    double xi, eta, rho, theta;
    xi = x + sin(phi);
    eta = y - 1. - cos(phi);
    polar(-eta, xi, rho, t);
    if (rho >= 2) {\
        u = 2 - rho;
        v = mod2pi(t + M_PI_2 - phi);
        if (t >= -EPS && u <= EPS && v <= EPS) {
            return true;
        }
    }
    return false;
}

void CSC(double x, double y, double phi, ReedsSheppPath &path)
{
    double t, u, v, length;
    // L+S+L+
    if (LpSpLp(x, y, phi, t, u, v) &&
            (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({L, S, L}, t, u, v, length);
    }
    // L-S-L-
    if (LpSpLp(-x, y, -phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({L, S, L}, -t, -u, -v, length);
    }
    // R+S+R+
    if (LpSpLp(x, -y, -phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({R, S, R}, t, u, v, length);
    }
    // R-S-R-
    if (LpSpLp(-x, -y, phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({R, S, R}, -t, -u, -v, length);
    }

    // L+S+R+
    if (LpSpRp(x, y, phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({L, S, R}, t, u, v, length);
    }
    // L-S-R-
    if (LpSpRp(-x, y, -phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({L, S, R}, -t, -u, -v, length);
    }
    // R+S+L+
    if (LpSpRp(x, -y, -phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({R, S, L}, t, u, v, length);
    }
    // R-S-L-
    if (LpSpRp(-x, -y, phi, t, u, v) &&
        (length = abs(t) + abs(u) + abs(v)) < path.m_length) {
        path = ReedsSheppPath({R, S, L}, -t, -u, -v, length);
    }
}


double getReedsSheppDistance(double x_start, double y_start, double phi_start,
                             double x_goal, double y_goal, double phi_goal,
                             double radius)
{
    ReedsShepp rs({x_start, y_start, phi_start},
                  {x_goal, y_goal, phi_goal}, radius);
    return 1.0;
}


PYBIND11_MODULE(reeds_shepp, handle)
{
    handle.doc() = "Get the shortest Reeds-Shepp distance.";

    handle.def("get_distance", &getReedsSheppDistance,
               "Get the shortest Reeds-Shepp distance.");
}
