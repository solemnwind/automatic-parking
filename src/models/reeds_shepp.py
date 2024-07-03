# import numpy as np
import numpy as np
from numpy import sin, cos, arcsin, arctan2, pi, sqrt, inf, abs
from enum import Enum

EPS = 1e-10


# util functions and data structures
def polar(x: float, y: float) -> tuple[float, float]:
    r = sqrt(x ** 2 + y ** 2)
    phi = arctan2(y, x)
    return r, phi


def mod2pi(theta: float) -> float:
    phi = theta % (2 * pi)
    if phi > pi:
        phi -= 2 * pi
    elif phi < -pi:
        phi += 2 * pi
    return phi


class RSWord(Enum):
    S = 0  # Straight
    L = 1  # Left
    R = 2  # Right


rs_types = {
    "RSR": [RSWord.R, RSWord.S, RSWord.R],
    "LSL": [RSWord.L, RSWord.S, RSWord.L],
    "LSR": [RSWord.L, RSWord.S, RSWord.R],
    "RSL": [RSWord.R, RSWord.S, RSWord.L],
    "RLR": [RSWord.R, RSWord.L, RSWord.R],
    "LRL": [RSWord.L, RSWord.R, RSWord.L],
}


class ReedsShepp:
    def __init__(self, x: float, y: float, phi: float):
        forms = [CSC, CCC, CCCC, CCSC, CCSCC]
        self.path: ReedsSheppPath or None = None
        l_min: float = inf
        for f in forms:
            p, l = f(x, y, phi)
            if l < l_min:
                l_min = l
                self.path = p

        self.length: float = l_min


class ReedsSheppPath:
    def __init__(self, path_type: list[RSWord], t: float, u: float, v: float, w: float = None, x: float = None):
        self.type = path_type
        self.t = t
        self.u = u
        self.v = v
        self.w = w
        self.x = x


def LpSpLp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    u, t = polar(x - sin(phi), y - 1.0 + cos(phi))
    if t >= -EPS:
        v = mod2pi(phi - t)
        if v >= -EPS:
            return True, t, u, v
    return False, 0, 0, 0


def LpSpRp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    u1, t1 = polar(x + sin(phi), y - 1.0 - cos(phi))
    if u1 ** 2 >= 4:
        u = sqrt(u1 ** 2 - 4)
        theta = arctan2(2, u)
        t = mod2pi(t1 + theta)
        v = mod2pi(t - phi)
        return True, t, u, v
    return False, 0, 0, 0


def LpRnL(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x - sin(phi)
    eta = y - 1 + cos(phi)
    u1, theta = polar(xi, eta)
    if u1 <= 4:
        u = 2 * arcsin(u1 / 4)
        t = mod2pi(theta - u / 2 + pi)
        v = mod2pi(phi - u - t)
        if t >= -EPS and u >= -EPS:
            return True, t, u, v
    return False, 0, 0, 0


def CSC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    t: float
    u: float
    v: float
    l_tot: float
    l_min: float = inf
    path: ReedsSheppPath or None = None
    # L+S+L+
    is_path, t, u, v = LpSpLp(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSL"], t, u, v)

    # L-S-L-
    is_path, t, u, v = LpSpLp(-x, y, -phi)  # time flip
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSL"], -t, -u, -v)

    # R+S+R+
    is_path, t, u, v = LpSpLp(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSR"], t, u, v)

    # R-S-R-
    is_path, t, u, v = LpSpLp(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSR"], -t, -u, -v)

    # L+S+R+
    is_path, t, u, v = LpSpRp(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSR"], t, u, v)

    # L-S-R-
    is_path, t, u, v = LpSpRp(-x, y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSR"], -t, -u, -v)

    # R+S+L+
    is_path, t, u, v = LpSpRp(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSL"], t, u, v)

    # R-S-L-
    is_path, t, u, v = LpSpRp(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSL"], -t, -u, -v)

    return path, l_min


def CCC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    t: float
    u: float
    v: float
    l_tot: float
    l_min: float = inf
    path: ReedsSheppPath or None = None
    # L+R-L+
    is_path, t, u, v = LpRnL(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRL"], t, u, v)

    # L-R+L-
    is_path, t, u, v = LpRnL(-x, y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRL"], -t, -u, -v)

    # R+L-R+
    is_path, t, u, v = LpRnL(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLR"], t, u, v)

    # R-L+R-
    is_path, t, u, v = LpRnL(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLR"], -t, -u, -v)

    return path, l_min


def CCCC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    t: float
    u: float
    v: float
    w: float
    l_tot: float
    l_min: float = inf
    path: ReedsSheppPath or None = None
    return path, l_min



def CCSC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    t: float
    u: float
    v: float
    w: float
    l_tot: float
    l_min: float = inf
    path: ReedsSheppPath or None = None
    return path, l_min


def CCSCC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    t: float
    u: float
    v: float
    w: float
    x: float
    l_tot: float
    l_min: float = inf
    path: ReedsSheppPath or None = None
    return path, l_min


if __name__ == "__main__":
    # path, l = CSC(2, 3, 0)
    # print(path.type)
    # print(path.t / pi * 180, path.u, path.v / pi * 180)
    # print(l)
    rs = ReedsShepp(-1, 1, pi/2)
    print(rs.path.type)
    print(rs.path.t, rs.path.u, rs.path.v)
    print(rs.length)
