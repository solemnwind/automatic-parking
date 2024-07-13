#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <pybind11/pybind11.h>
#include <iostream>
#include <utility>
#include <vector>
#include <array>
#include <map>
#include <string>

#define EPS 1e-8

enum RSWord {S, R, L};
typedef std::array<double, 3> Pose_t;
typedef std::vector<RSWord> RSPathType;


double mod2pi(double theta);
void polar(double x, double y, double &r, double &phi);
void tw(double u, double u1, double xi, double eta, double phi);

class ReedsSheppPath
{
public:
    RSPathType m_type;
    std::array<double, 5> m_lengths;
    double m_length;

public:
    ReedsSheppPath(RSPathType pt, double t, double u, double v, double w, double x, double length)
            : m_type(std::move(pt)), m_lengths{t, u, v, w, x}, m_length(length) {};
    ReedsSheppPath(RSPathType pt, double t, double u, double v, double w, double length)
            : ReedsSheppPath(std::move(pt), t, u, v, w, 0., length) {};
    ReedsSheppPath(RSPathType pt, double t, double u, double v, double length)
            : ReedsSheppPath(std::move(pt), t, u, v, 0., 0., length) {};
};

class ReedsShepp
{
public:
    double m_distance;

private:
    Pose_t m_start, m_goal;
    double m_radius;
    double m_x, m_y, m_phi;
    ReedsSheppPath path;

public:
    ReedsShepp(Pose_t start, Pose_t goal, double radius);

private:
    static Pose_t transformToOrigin(Pose_t &start, Pose_t &goal);
};

bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v);
bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v);
bool LpRnL(double x, double y, double phi, double &t, double &u, double &v);
bool LpRpuLnuRn(double x, double y, double phi, double &t, double &u, double &v);
bool LpRnuLnuRp(double x, double y, double phi, double &t, double &u, double &v);
bool LpRnSnLn(double x, double y, double phi, double &t, double &u, double &v);
bool LpRnSnRn(double x, double y, double phi, double &t, double &u, double &v);

void CSC(double x, double y, double phi, ReedsSheppPath &path);


double getReedsSheppDistance(double x_start, double y_start, double phi_start,
                             double x_goal, double y_goal, double phi_goal,
                             double radius);

#endif //REEDS_SHEPP_H
