#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <pybind11/pybind11.h>
#include <iostream>
#include <vector>
#include <array>

#define EPS 1e-8

typedef std::array<double, 3> Pose_t;

double mod2pi(double theta);
void polar(double x, double y, double &r, double &phi);

class ReedsShepp
{
public:
    double m_distance;

private:
    Pose_t m_start, m_goal;
    double m_radius;
    double m_x, m_y, m_phi;


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


double getReedsSheppDistance(double x_start, double y_start, double phi_start,
                             double x_goal, double y_goal, double phi_goal,
                             double radius);

#endif //REEDS_SHEPP_H
