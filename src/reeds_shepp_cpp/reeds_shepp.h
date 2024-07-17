#ifndef REEDS_SHEPP_H
#define REEDS_SHEPP_H

#define _USE_MATH_DEFINES
#include <cmath>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <utility>
#include <vector>
#include <array>
#include <limits>

#define INF std::numeric_limits<double>::infinity()
#define EPS 1e-8

enum RSWord {S = 0, L, R};
typedef std::array<double, 3> Pose_t;
typedef std::vector<RSWord> RSPathType;


double mod2pi(double theta);
void polar(double x, double y, double &r, double &phi);
void tw(double u, double u1, double xi, double eta, double phi, double &tau, double &omega);

struct ReedsSheppPath
{
    RSPathType m_type;
    std::array<double, 5> m_lengths;
    double m_radius = 1.;
    double m_distance;

    ReedsSheppPath(RSPathType pt, double t, double u, double v, double w, double x, double distance)
            : m_type(std::move(pt)), m_lengths{t, u, v, w, x}, m_distance(distance) {};
    ReedsSheppPath(RSPathType pt, double t, double u, double v, double w, double distance)
            : ReedsSheppPath(std::move(pt), t, u, v, w, 0., distance) {};
    ReedsSheppPath(RSPathType pt, double t, double u, double v, double distance)
            : ReedsSheppPath(std::move(pt), t, u, v, 0., 0., distance) {};
    ReedsSheppPath() : m_lengths{INF, 0., 0., 0., 0.},  m_distance(INF) {};

    ReedsSheppPath& operator*=(double a);

    double getDistance() const { return m_distance; };
};

class ReedsShepp
{
private:
    Pose_t m_start, m_goal;
    double m_radius;
    double m_x, m_y, m_phi;
    ReedsSheppPath m_path;

public:
    ReedsShepp(Pose_t start, Pose_t goal, double radius);

    double getDistance() const { return m_path.m_distance; };
    ReedsSheppPath getPath() const { return m_path; };
    void printPathInfo() const;

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
bool LpRnSnLnRp(double x, double y, double phi, double &t, double &u, double &v);

void CSC(double x, double y, double phi, ReedsSheppPath &path);
void CCC(double x, double y, double phi, ReedsSheppPath &path);
void CCCC(double x, double y, double phi, ReedsSheppPath &path);
void CCSC(double x, double y, double phi, ReedsSheppPath &path);
void CCSCC(double x, double y, double phi, ReedsSheppPath &path);


double getReedsSheppDistance(Pose_t start, Pose_t goal, double radius);

#endif //REEDS_SHEPP_H
