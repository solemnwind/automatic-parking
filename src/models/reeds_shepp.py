# import numpy as np
import numpy as np
from numpy import sin, cos, arcsin, arctan2, pi, sqrt, inf, abs
from enum import Enum

EPS = 1e-10
RAD = 180 / pi
DEG = pi / 180


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
        self.lengths = [t, u, v, w, x]


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
            return True, t, -u, v
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
    import argparse
    import matplotlib.pyplot as plt
    from matplotlib.patches import Arc

    parser = argparse.ArgumentParser(
        prog='reeds_shepp.py',
        description='Computes the optimal Reeds-Shepp path from (0, 0, 0) to (x, y, phi)'
    )
    parser.add_argument('x', type=float, help='x coordinate')
    parser.add_argument('y', type=float, help='y coordinate')
    parser.add_argument('phi', type=float, help='orientation phi (degree)')
    args = parser.parse_args()

    p = [args.x, args.y, args.phi * DEG]
    rs = ReedsShepp(p[0], p[1], p[2])

    pos = [0, 0, 0]
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    # plt.grid()

    # Plot the path
    for i in range(len(rs.path.type)):
        action = rs.path.type[i]
        amount = rs.path.lengths[i]
        # Show the path in blue if goes forward,
        # in red if goes backward.
        linecolor = 'b' if amount >= 0 else 'r'

        match action:
            case RSWord.S:
                pos_next = [pos[0] + amount * cos(pos[2]), pos[1] + amount * sin(pos[2]), pos[2]]
                ax.plot([pos[0], pos_next[0]], [pos[1], pos_next[1]], linecolor, linewidth=2)
                pos = pos_next
            case RSWord.L:
                centric = (pos[0] - sin(pos[2]), pos[1] + cos(pos[2]))
                phi_next = mod2pi(pos[2] + amount)
                pos_next = [centric[0] + sin(phi_next), centric[1] - cos(phi_next), phi_next]
                theta1 = pos[2] * RAD if amount >= 0 else phi_next * RAD
                theta2 = phi_next * RAD if amount >= 0 else pos[2] * RAD
                ax.add_patch(Arc(centric, 2, 2, angle=-90,
                                 theta1=theta1, theta2=theta2, edgecolor=linecolor, linewidth=2))
                pos = pos_next
            case RSWord.R:
                centric = (pos[0] + sin(pos[2]), pos[1] - cos(pos[2]))
                phi_next = mod2pi(pos[2] - amount)
                pos_next = [centric[0] - sin(phi_next), centric[1] + cos(phi_next), phi_next]
                theta1 = pos[2] * RAD if amount < 0 else phi_next * RAD
                theta2 = phi_next * RAD if amount < 0 else pos[2] * RAD
                ax.add_patch(Arc(centric, 2, 2, angle=90,
                                 theta1=theta1, theta2=theta2, edgecolor=linecolor, linewidth=2))
                pos = pos_next

    # Plot the start and end
    plt.scatter(0, 0, marker='.', color='gold', s=12)
    plt.scatter(p[0], p[1], marker='*', color='g', s=12)
    plt.quiver([0, p[0]], [0, p[1]], [1, cos(p[2])], [0, sin(p[2])],
               color=['gold', 'g'])

    ax.autoscale_view()
    plt.show()
