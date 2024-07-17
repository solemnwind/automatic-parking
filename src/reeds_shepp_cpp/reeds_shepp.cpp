#include "reeds_shepp.h"

namespace py = pybind11;
using namespace std;


double mod2pi(double theta)
{
    double phi = fmod(theta, 2 * M_PI);
    if (phi > M_PI) {
        phi -= 2 * M_PI;
    } else if (phi <= -M_PI) {
        phi += 2 * M_PI;
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

//====================== class ReedsSheppPath ======================

ReedsSheppPath& ReedsSheppPath::operator*=(double a)
{
    for (double& l : m_lengths) {
        l *= a;
    }
    m_radius *= a;
    m_distance *= a;
    return *this;
}

//---------------------- class ReedsSheppPath ----------------------

//====================== class ReedsShepp ======================
ReedsShepp::ReedsShepp(Pose_t start, Pose_t goal, double radius)
    : m_start(start), m_goal(goal), m_radius(radius)
{
    Pose_t normalizedGoal = transformToOrigin(m_start, m_goal);
    m_x = normalizedGoal[0];
    m_y = normalizedGoal[1];
    m_phi = normalizedGoal[2];

    double x = m_x / m_radius;
    double y = m_y / m_radius;

    CSC(x, y, m_phi, m_path);
    CCC(x, y, m_phi, m_path);
    CCSC(x, y, m_phi, m_path);
    CCCC(x, y, m_phi, m_path);
    CCSCC(x, y, m_phi, m_path);

    m_path *= m_radius;
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
    eta = y - 1. + cos(phi);
    polar(xi, eta, u1, theta);
    if (u1 <= 4.) {
        u = -2. * asin(u1 / 4.);
        t = mod2pi(theta + u / 2. + M_PI);
        v = mod2pi(phi + u - t);
        if (t >= -EPS && u <= EPS) {
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
    double xi, eta, rho;
    xi = x + sin(phi);
    eta = y - 1. - cos(phi);
    polar(-eta, xi, rho, t);
    if (rho >= 2) {
        u = 2 - rho;
        v = mod2pi(t + M_PI_2 - phi);
        if (t >= -EPS && u <= EPS && v <= EPS) {
            return true;
        }
    }
    return false;
}

bool LpRnSnLnRp(double x, double y, double phi, double &t, double &u, double &v)
{
    double xi, eta, rho, theta;
    xi = x + sin(phi);
    eta = y - 1. -cos(phi);
    polar(eta, xi, rho, theta);
    if (rho >= 2) {
        t = mod2pi(theta - acos(-2. / rho));
        if (t > -EPS) {
            u = 4. - (xi + 2. * cos(t)) / sin(t);
            v = mod2pi(t - phi);
            if (t >= -EPS && u <= EPS && v >= -EPS) {
                return true;
            }
        }
    }
    return false;
}

void CSC(double x, double y, double phi, ReedsSheppPath &path)
{
    double t, u, v, distance;
    // L+S+L+
    if (LpSpLp(x, y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, S, L}, t, u, v, distance);
    }
    // L-S-L-
    if (LpSpLp(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, S, L}, -t, -u, -v, distance);
    }
    // R+S+R+
    if (LpSpLp(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, S, R}, t, u, v, distance);
    }
    // R-S-R-
    if (LpSpLp(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, S, R}, -t, -u, -v, distance);
    }

    // L+S+R+
    if (LpSpRp(x, y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, S, R}, t, u, v, distance);
    }
    // L-S-R-
    if (LpSpRp(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, S, R}, -t, -u, -v, distance);
    }
    // R+S+L+
    if (LpSpRp(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, S, L}, t, u, v, distance);
    }
    // R-S-L-
    if (LpSpRp(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, S, L}, -t, -u, -v, distance);
    }
}

void CCC(double x, double y, double phi, ReedsSheppPath &path)
{
    double t, u, v, distance;
    // C|C|C and C|CC
    // L+R-L
    if (LpRnL(x, y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L}, t, u, v, distance);
    }
    // L-R+L
    if (LpRnL(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L}, -t, -u, -v, distance);
    }
    // R+L-R
    if (LpRnL(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R}, t, u, v, distance);
    }
    // R-L+R
    if (LpRnL(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R}, -t, -u, -v, distance);
    }
    // CC|C
    double xb, yb;
    xb = x * cos(phi) + y * sin(phi);
    yb = x * sin(phi) - y * cos(phi);
    // LR-L+
    if (LpRnL(xb, yb, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L}, v, u, t, distance);
    }
    // LR+L-
    if (LpRnL(-xb, yb, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L}, -v, -u, -t, distance);
    }
    // RL-R+
    if (LpRnL(xb, -yb, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R}, v, u, t, distance);
    }
    // RL+R-
    if (LpRnL(-xb, -yb, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R}, -v, -u, -t, distance);
    }
}

void CCCC(double x, double y, double phi, ReedsSheppPath &path)
{
    double t, u, v, distance;
    // CCu|CuC
    // L+R+L-R-
    if (LpRpuLnuRn(x, y, phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L, R}, t, u, -u, v, distance);
    }
    // L-R-L+R+
    if (LpRpuLnuRn(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L, R}, -t, -u, u, -v, distance);
    }
    // R+L+R-L-
    if (LpRpuLnuRn(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R, L}, t, u, -u, v, distance);
    }
    // R-L-R+L+
    if (LpRpuLnuRn(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R, L}, -t, -u, u, -v, distance);
    }

    // C|CuCu|C
    // L+R-L-R+
    if (LpRnuLnuRp(x, y, phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L, R}, t, u, u, v, distance);
    }
    // L-R+L+R-
    if (LpRnuLnuRp(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({L, R, L, R}, -t, -u, -u, -v, distance);
    }
    // R+L-R-L+
    if (LpRnuLnuRp(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R, L}, t, u, u, v, distance);
    }
    // R-L+R+L-
    if (LpRnuLnuRp(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + 2. * abs(u) + abs(v)) < path.m_distance) {
        path = ReedsSheppPath({R, L, R, L}, -t, -u, -u, -v, distance);
    }
}

void CCSC(double x, double y, double phi, ReedsSheppPath &path)
{
    double t, u, v, distance;
    // C|C_{pi/2}SC
    // L+R-S-L-
    if (LpRnSnLn(x, y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, R, S, L}, t, -M_PI_2, u, v, distance);
    }
    // L-R+S+L+
    if (LpRnSnLn(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, R, S, L}, -t, M_PI_2, -u, -v, distance);
    }
    // R+L-S-R-
    if (LpRnSnLn(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, L, S, R}, t, -M_PI_2, u, v, distance);
    }
    // R-L+S+R+
    if (LpRnSnLn(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, L, S, R}, -t, M_PI_2, -u, -v, distance);
    }

    // L+R-S-R-
    if (LpRnSnRn(x, y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, R, S, R}, t, -M_PI_2, u, v, distance);
    }
    // L-R+S+R+
    if (LpRnSnRn(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, R, S, R}, -t, M_PI_2, -u, -v, distance);
    }
    // R+L-S-L-
    if (LpRnSnRn(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, L, S, L}, t, -M_PI_2, u, v, distance);
    }
    // R-L+S+L+
    if (LpRnSnRn(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, L, S, L}, -t, M_PI_2, -u, -v, distance);
    }

    // CS|C_{pi/2}C
    double xb = x * cos(phi) + y * sin(phi);
    double yb = x * sin(phi) - y * cos(phi);
    // L-S-R-L+
    if (LpRnSnLn(xb, yb, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, S, R, L}, v, u, -M_PI_2, t, distance);
    }
    // L+S+R+L-
    if (LpRnSnLn(-xb, yb, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, S, R, L}, -v, -u, M_PI_2, -t, distance);
    }
    // R-S-L-R+
    if (LpRnSnLn(xb, -yb, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, S, L, R}, v, u, -M_PI_2, t, distance);
    }
    // R+S+L+R-
    if (LpRnSnLn(-xb, -yb, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, S, L, R}, -v, -u, M_PI_2, -t, distance);
    }

    // R-S-R-L+
    if (LpRnSnRn(xb, yb, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, S, R, L}, v, u, -M_PI_2, t, distance);
    }
    // R+S+R+L-
    if (LpRnSnRn(-xb, yb, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({R, S, R, L}, -v, -u, M_PI_2, -t, distance);
    }
    // L-S-L-R+
    if (LpRnSnRn(xb, -yb, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, S, L, R}, v, u, -M_PI_2, t, distance);
    }
    // L+S+L+R-
    if (LpRnSnRn(-xb, -yb, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI_2) < path.m_distance) {
        path = ReedsSheppPath({L, S, L, R}, -v, -u, M_PI_2, -t, distance);
    }
}

void CCSCC(double x, double y, double phi, ReedsSheppPath &path)
{
    double t, u, v, distance;
    // L+R-S-L-R+
    if (LpRnSnLnRp(x, y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI) < path.m_distance) {
        path = ReedsSheppPath({L, R, S, L, R}, t, -M_PI_2, u, -M_PI_2, v, distance);
    }
    // L-R+S+L+R-
    if (LpRnSnLnRp(-x, y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI) < path.m_distance) {
        path = ReedsSheppPath({L, R, S, L, R}, -t, M_PI_2, -u, M_PI_2, -v, distance);
    }
    // R+L-S-R-L+
    if (LpRnSnLnRp(x, -y, -phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI) < path.m_distance) {
        path = ReedsSheppPath({R, L, S, R, L}, t, -M_PI_2, u, -M_PI_2, v, distance);
    }
    // R-L+S+R+L-
    if (LpRnSnLnRp(-x, -y, phi, t, u, v) &&
        (distance = abs(t) + abs(u) + abs(v) + M_PI) < path.m_distance) {
        path = ReedsSheppPath({R, L, S, R, L}, -t, M_PI_2, -u, M_PI_2, -v, distance);
    }
}


double getReedsSheppDistance(Pose_t startPose, Pose_t goalPose, double radius)
{
    ReedsShepp rs(startPose, goalPose, radius);
    return rs.getDistance();
}

ReedsSheppPath getReedsSheppPath(Pose_t startPose, Pose_t goalPose, double radius)
{
    ReedsShepp rs(startPose, goalPose, radius);
    return rs.getPath();
}

Pose_t interpolatePoseAtDistance(Pose_t startPose, ReedsSheppPath &path, double travelledDistance)
{
    double t = clamp(travelledDistance, 0., path.m_distance);
    Pose_t pose = startPose;

    for (unsigned long i = 0; i < path.m_type.size() && t > 0; ++i) {
        // Get the travelling distance for this segment
        double x = path.m_lengths[i];
        if (x > EPS) {
            x = min(x, t);
            t -= x;
        } else if (x < -EPS) {
            x = max(x, -t);
            t += x;
        } else {
            continue;
        }

        switch (path.m_type[i]) {
            case S:
                pose = Pose_t{pose[0] + x * cos(pose[2]),
                              pose[1] + x * sin(pose[2]),
                              pose[2]};
                break;
            case L:
                x = mod2pi(pose[2] + x / path.m_radius);  // New phi angle
                pose = Pose_t{pose[0] + path.m_radius * (sin(x) - sin(pose[2])),
                              pose[1] + path.m_radius * (cos(pose[2]) - cos(x)),
                              x};
                break;
            case R:
                x = mod2pi(pose[2] - x / path.m_radius);  // New phi angle
                pose = Pose_t{pose[0] + path.m_radius * (sin(pose[2]) - sin(x)),
                              pose[1] + path.m_radius * (cos(x) - cos(pose[2])),
                              x};
                break;
        }
    }
    return pose;
}

PYBIND11_MODULE(_reeds_shepp, m)
{
    m.doc() = "C++ Reeds-Shepp model and interface";

    py::enum_<RSWord>(m, "RSWord")
        .value("S", S)
        .value("L", L)
        .value("R", R)
        .export_values();

    py::class_<ReedsSheppPath>(m, "ReedsSheppPath")
        .def(py::init<>())
        .def_readonly("type", &ReedsSheppPath::m_type)
        .def_readonly("lengths", &ReedsSheppPath::m_lengths)
        .def_readonly("radius", &ReedsSheppPath::m_radius)
        .def_readonly("distance", &ReedsSheppPath::m_distance);

    m.def("get_reeds_shepp_distance", &getReedsSheppDistance,
          "Get the shortest Reeds-Shepp distance.");

    m.def("get_reeds_shepp_path", &getReedsSheppPath,
          "Get the shortest Reeds-Shepp path.");

    m.def("interpolate_reeds_shepp_path", &interpolatePoseAtDistance,
          "Interpolate at the Reeds-Shepp path.");
}
