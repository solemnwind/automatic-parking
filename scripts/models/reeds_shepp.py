import numpy as np
from numpy import sin, cos, arcsin, arccos, arctan2, pi, sqrt, inf, abs
from enum import Enum
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

Pose_t = tuple[float, float, float]
EPS = 1e-10
RAD = 180 / pi
DEG = pi / 180
pi_2 = pi / 2


# util functions and data structures
def polar(x: float, y: float) -> tuple[float, float]:
    r = sqrt(x ** 2 + y ** 2)
    phi = arctan2(y, x)
    return r, phi


def mod2pi(theta: float) -> float:
    """ theta -> (-pi, pi]"""
    phi = theta % (2 * pi)
    if phi > pi:
        phi -= 2 * pi
    elif phi <= -pi:
        phi += 2 * pi
    return phi


def transform2origin(start: Pose_t, goal: Pose_t) -> Pose_t:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    theta = -start[2]
    x = dx * cos(theta) - dy * sin(theta)
    y = dx * sin(theta) + dy * cos(theta)
    phi = mod2pi(goal[2] - start[2])
    return x, y, phi


# Reeds-Shepp "words": L, S, R
class RSWord(Enum):
    S = 0  # Straight
    L = 1  # Left
    R = 2  # Right


rs_types = {
    "RSR":   [RSWord.R, RSWord.S, RSWord.R],
    "LSL":   [RSWord.L, RSWord.S, RSWord.L],
    "LSR":   [RSWord.L, RSWord.S, RSWord.R],
    "RSL":   [RSWord.R, RSWord.S, RSWord.L],
    "RLR":   [RSWord.R, RSWord.L, RSWord.R],
    "LRL":   [RSWord.L, RSWord.R, RSWord.L],
    "LRLR":  [RSWord.L, RSWord.R, RSWord.L, RSWord.R],
    "RLRL":  [RSWord.R, RSWord.L, RSWord.R, RSWord.L],
    "LRSL":  [RSWord.L, RSWord.R, RSWord.S, RSWord.L],
    "RLSR":  [RSWord.R, RSWord.L, RSWord.S, RSWord.R],
    "LRSR":  [RSWord.L, RSWord.R, RSWord.S, RSWord.R],
    "RLSL":  [RSWord.R, RSWord.L, RSWord.S, RSWord.L],
    "LSRL":  [RSWord.L, RSWord.S, RSWord.R, RSWord.L],
    "RSLR":  [RSWord.R, RSWord.S, RSWord.L, RSWord.R],
    "RSRL":  [RSWord.R, RSWord.S, RSWord.R, RSWord.L],
    "LSLR":  [RSWord.L, RSWord.S, RSWord.L, RSWord.R],
    "LRSLR": [RSWord.L, RSWord.R, RSWord.S, RSWord.L, RSWord.R],
    "RLSRL": [RSWord.R, RSWord.L, RSWord.S, RSWord.R, RSWord.L]
}


class ReedsShepp:
    def __init__(self, start: Pose_t = (0, 0, 0), goal: Pose_t = (1, 0, 0), radius: float = 1.0):
        assert radius > 0
        self.start = start
        self.goal = goal
        self.radius = radius
        x, y, phi = transform2origin(start, goal)
        forms = [CSC, CCC, CCCC, CCSC, CCSCC]
        self.path: ReedsSheppPath or None = None
        l_min: float = inf
        for f in forms:
            p, l = f(x / radius, y / radius, phi)
            if l < l_min:
                l_min = l
                self.path = p
        self.path *= radius
        self.distance: float = l_min * radius

    def draw(self, fig: plt.Figure = None, ax: plt.Axes = None) -> tuple[plt.Figure, plt.Axes]:
        if fig is None or ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111)
        ax.set_aspect('equal')

        pos = self.start
        r = self.radius
        d = 2 * r
        # Plot the path
        for i in range(len(self.path.type)):
            action = self.path.type[i]
            amount = self.path.lengths[i]
            linecolor = 'b' if amount >= 0 else 'r'  # Show positive path in blue and negative path in red

            match action:
                case RSWord.S:
                    pos_next = [pos[0] + amount * cos(pos[2]), pos[1] + amount * sin(pos[2]), pos[2]]
                    ax.plot([pos[0], pos_next[0]], [pos[1], pos_next[1]], linecolor, linewidth=2)
                    pos = pos_next
                case RSWord.L:
                    center = (pos[0] - r * sin(pos[2]), pos[1] + r * cos(pos[2]))
                    phi_next = mod2pi(pos[2] + amount / self.radius)
                    pos_next = [center[0] + r * sin(phi_next), center[1] - r * cos(phi_next), phi_next]
                    theta1 = pos[2] * RAD if amount >= 0 else phi_next * RAD
                    theta2 = phi_next * RAD if amount >= 0 else pos[2] * RAD
                    ax.add_patch(Arc(center, d, d, angle=-90,
                                     theta1=theta1, theta2=theta2, edgecolor=linecolor, linewidth=2))
                    pos = pos_next
                case RSWord.R:
                    center = (pos[0] + r * sin(pos[2]), pos[1] - r * cos(pos[2]))
                    phi_next = mod2pi(pos[2] - amount / self.radius)
                    pos_next = [center[0] - r * sin(phi_next), center[1] + r * cos(phi_next), phi_next]
                    theta1 = pos[2] * RAD if amount < 0 else phi_next * RAD
                    theta2 = phi_next * RAD if amount < 0 else pos[2] * RAD
                    ax.add_patch(Arc(center, d, d, angle=90,
                                     theta1=theta1, theta2=theta2, edgecolor=linecolor, linewidth=2))
                    pos = pos_next

        # Plot the start and goal poses
        plt.scatter(self.start[0], self.start[1], marker='.', color='gold', s=12)
        plt.scatter(self.goal[0], self.goal[1], marker='*', color='g', s=12)
        plt.quiver([self.start[0], self.goal[0]], [self.start[1], self.goal[1]],
                   [cos(self.start[2]), cos(self.goal[2])], [sin(self.start[2]), sin(self.goal[2])],
                   color=['gold', 'g'])

        ax.autoscale_view()

        return fig, ax


class ReedsSheppPath:
    def __init__(self, path_type: list[RSWord],
                 t: float, u: float, v: float, w: float = None, x: float = None):
        self.type = path_type
        self.lengths: list[float or None] = [t, u, v, w, x]
        self.radius: float = 1.0
        self.distance: float = sum(abs(x) for x in self.lengths if x is not None)

    def __imul__(self, other):
        if isinstance(other, float) or isinstance(other, int):
            self.radius *= other
            self.distance *= other
            for i in range(len(self.type)):
                self.lengths[i] *= other
            return self
        else:
            raise TypeError("Unsupported type for *=")


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


def CSC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
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


def CCC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    l_min: float = inf
    path: ReedsSheppPath or None = None
    """ C|C|C and C|CC """
    # L+R-L
    is_path, t, u, v = LpRnL(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRL"], t, u, v)

    # L-R+L
    is_path, t, u, v = LpRnL(-x, y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRL"], -t, -u, -v)

    # R+L-R
    is_path, t, u, v = LpRnL(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLR"], t, u, v)

    # R-L+R
    is_path, t, u, v = LpRnL(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLR"], -t, -u, -v)

    """ CC|C """
    # Reverse of C|CC
    xb = x * cos(phi) + y * sin(phi)
    yb = x * sin(phi) - y * cos(phi)
    # L-R-L+
    is_path, t, u, v = LpRnL(xb, yb, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRL"], v, u, t)

    # L+R+L-
    is_path, t, u, v = LpRnL(-xb, yb, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRL"], -v, -u, -t)

    # R-L-R+
    is_path, t, u, v = LpRnL(xb, -yb, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLR"], v, u, t)

    # R+L+R-
    is_path, t, u, v = LpRnL(-xb, -yb, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLR"], -v, -u, -t)

    return path, l_min


def tw(u, u1, xi, eta, phi) -> tuple[float, float]:
    delta = mod2pi(u - u1)
    A = sin(u) - sin(delta)
    B = cos(u) - cos(delta) - 1
    t1 = arctan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2 * cos(delta) - 2 * cos(u1) - 2 * cos(u) + 3
    tau = mod2pi(t1 + pi) if t2 < 0 else mod2pi(t1)
    omega = mod2pi(tau - u + u1 - phi)
    return tau, omega


# (8.7)
def LpRpuLnuRn(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + sin(phi)
    eta = y - 1 - cos(phi)
    rho = (2 + sqrt(xi**2 + eta**2)) / 4
    if rho <= 1:
        u = arccos(rho)
        t, v = tw(u, -u, xi, eta, phi)
        if t >= -EPS and v <= EPS:
            return True, t, u, v
    return False, 0, 0, 0


# (8.8)
def LpRnuLnuRp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + sin(phi)
    eta = y - 1 - cos(phi)
    rho = (20 - xi**2 - eta**2) / 16
    if 0 <= rho <= 1:
        u = -arccos(rho)
        t, v = tw(u, u, xi, eta, phi)
        if t >= -EPS and v >= -EPS:
            return True, t, u, v
    return False, 0, 0, 0


def CCCC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    l_min: float = inf
    path: ReedsSheppPath or None = None
    """ CCu|CuC """
    # L+R+L-R-
    is_path, t, u, v = LpRpuLnuRn(x, y, phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRLR"], t, u, -u, v)
    # L-R-L+R+
    is_path, t, u, v = LpRpuLnuRn(-x, y, -phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRLR"], -t, -u, u, -v)
    # R+L+R-L-
    is_path, t, u, v = LpRpuLnuRn(x, -y, -phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLRL"], t, u, -u, v)
    # R-L-R+L+
    is_path, t, u, v = LpRpuLnuRn(-x, -y, phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLRL"], -t, -u, u, -v)

    """ C|CuCu|C """
    # L+R-L-R+
    is_path, t, u, v = LpRnuLnuRp(x, y, phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRLR"], t, u, u, v)
    # L-R+L+R-
    is_path, t, u, v = LpRnuLnuRp(-x, y, -phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRLR"], -t, -u, -u, -v)
    # R+L-R-L+
    is_path, t, u, v = LpRnuLnuRp(x, -y, -phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLRL"], t, u, u, v)
    # R-L+R+L-
    is_path, t, u, v = LpRnuLnuRp(-x, -y, phi)
    l_tot = abs(t) + 2 * abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLRL"], -t, -u, -u, -v)

    return path, l_min


# (8.9)
def LpRnSnLn(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + sin(phi)
    eta = y - 1 - cos(phi)
    rho, theta = polar(-eta, xi)
    if rho >= 2:
        _, theta1 = polar(sqrt(rho**2 - 4), 2)
        t = mod2pi(theta - theta1)
        u = 2 - theta1
        v = mod2pi(phi - pi_2 - t)
        if t >= -EPS and u <= EPS and v <= EPS:
            return True, t, u, v
    return False, 0, 0, 0


# (8.10)
def LpRnSnRn(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + sin(phi)
    eta = y - 1 - cos(phi)
    rho, theta = polar(-eta, xi)
    if rho >= 2:
        t = theta
        u = 2 - rho
        v = mod2pi(t + pi_2 - phi)
        if t >= -EPS and u <= EPS and v <= EPS:
            return True, t, u, v
    return False, 0, 0, 0


def CCSC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    l_min: float = inf
    path: ReedsSheppPath or None = None
    """ C|C_{pi/2}SC """
    # LpRnSnLn
    is_path, t, u, v = LpRnSnLn(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRSL"], t, -pi_2, u, v)
    # LnRpSpLp
    is_path, t, u, v = LpRnSnLn(-x, y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRSL"], -t, pi_2, -u, -v)
    # RpLnSnRn
    is_path, t, u, v = LpRnSnLn(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLSR"], t, -pi_2, u, v)
    # RnLpSpRp
    is_path, t, u, v = LpRnSnLn(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLSR"], -t, pi_2, -u, -v)

    # LpRnSnRn
    is_path, t, u, v = LpRnSnRn(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRSR"], t, -pi_2, u, v)
    # LnRpSpRp
    is_path, t, u, v = LpRnSnRn(-x, y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRSR"], -t, pi_2, -u, -v)
    # RpLnSnLn
    is_path, t, u, v = LpRnSnRn(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLSL"], t, -pi_2, u, v)
    # RnLpSpLp
    is_path, t, u, v = LpRnSnRn(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLSL"], -t, pi_2, -u, -v)

    """ CS|C_{pi/2}C """
    xb = x * cos(phi) + y * sin(phi)
    yb = x * sin(phi) - y * cos(phi)
    # LnSnRnLp
    is_path, t, u, v = LpRnSnLn(xb, yb, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSRL"], v, u, -pi_2, t)
    # LpSpRpLn
    is_path, t, u, v = LpRnSnLn(-xb, yb, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSRL"], -v, -u, pi_2, -t)
    # RnSnLnRp
    is_path, t, u, v = LpRnSnLn(xb, -yb, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSLR"], v, u, -pi_2, t)
    # RpSpLpRn
    is_path, t, u, v = LpRnSnLn(-xb, -yb, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSLR"], -v, -u, pi_2, -t)

    # RnSnRnLp
    is_path, t, u, v = LpRnSnRn(xb, yb, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSRL"], v, u, -pi_2, t)
    # RpSpRpLn
    is_path, t, u, v = LpRnSnRn(-xb, yb, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RSRL"], -v, -u, pi_2, -t)
    # LnSnLnRp
    is_path, t, u, v = LpRnSnRn(xb, -yb, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSLR"], v, u, -pi_2, t)
    # LpSpLpRn
    is_path, t, u, v = LpRnSnRn(-xb, -yb, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LSLR"], -v, -u, pi_2, -t)

    return path, l_min + pi_2


# (8.11)
def LpRnSnLnRp(x: float, y: float, phi: float) -> tuple[bool, float, float, float]:
    xi = x + sin(phi)
    eta = y - 1 - cos(phi)
    rho, theta = polar(eta, xi)
    if rho >= 2:
        t = mod2pi(theta - arccos(-2 / rho))
        if t > 0:
            u = 4 - (xi + 2 * cos(t)) / sin(t)
            v = mod2pi(t - phi)
            if t >= -EPS and u <= EPS and v >= -EPS:
                return True, t, u, v
    return False, 0, 0, 0


def CCSCC(x: float, y: float, phi: float) -> tuple[ReedsSheppPath or None, float]:
    l_min: float = inf
    path: ReedsSheppPath or None = None
    # L+R-S-L-R+
    is_path, t, u, v = LpRnSnLnRp(x, y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRSLR"], t, -pi_2, u, -pi_2, v)
    # L-R+S+L+R-
    is_path, t, u, v = LpRnSnLnRp(-x, y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["LRSLR"], -t, pi_2, -u, pi_2, -v)
    # R+L-S-R-L+
    is_path, t, u, v = LpRnSnLnRp(x, -y, -phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLSRL"], t, -pi_2, u, -pi_2, v)
    # R-L+S+R+L-
    is_path, t, u, v = LpRnSnLnRp(-x, -y, phi)
    l_tot = abs(t) + abs(u) + abs(v)
    if is_path and l_tot < l_min:
        l_min = l_tot
        path = ReedsSheppPath(rs_types["RLSRL"], -t, pi_2, -u, pi_2, -v)

    return path, l_min + pi


def get_reeds_shepp_distance(start: Pose_t, goal: Pose_t, radius: float) -> float:
    return ReedsShepp(start, goal, radius).distance


def get_reeds_shepp_path(start: Pose_t, goal: Pose_t, radius: float) -> ReedsSheppPath:
    return ReedsShepp(start, goal, radius).path


def interpolate_reeds_shepp_path(start: Pose_t, path: ReedsSheppPath, d: float) -> Pose_t:
    d = np.clip(d, 0, path.distance)
    t = d
    pose = start
    r = path.radius
    # Plot the path
    for i in range(len(path.type)):
        if t <= 0:
            break

        amount = path.lengths[i]
        if -EPS <= amount <= EPS:
            continue
        if amount > 0:
            amount = min(amount, t)
            t -= amount
        else:
            amount = max(amount, -t)
            t += amount

        match path.type[i]:
            case RSWord.S:
                pos_next = [pose[0] + amount * cos(pose[2]), pose[1] + amount * sin(pose[2]), pose[2]]
                pose = pos_next
            case RSWord.L:
                phi_next = mod2pi(pose[2] + amount / path.radius)
                pos_next = [pose[0] + r * (sin(phi_next) - sin(pose[2])),
                            pose[1] + r * (-cos(phi_next) + cos(pose[2])),
                            phi_next]
                pose = pos_next
            case RSWord.R:
                phi_next = mod2pi(pose[2] - amount / path.radius)
                pos_next = [pose[0] + r * (sin(pose[2]) - sin(phi_next)),
                            pose[1] + r * (cos(phi_next) - cos(pose[2])),
                            phi_next]
                pose = pos_next
    return pose


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        prog='reeds_shepp.py',
        description='Computes the optimal Reeds-Shepp path from (0, 0, 0) to (x, y, phi)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('-s', '--start', nargs=3, type=float,
                        help='specify start pose (x, y, phi), phi is in Degree', default=(0, 0, 0))
    parser.add_argument('-g', '--goal', nargs=3, type=float,
                        help='specify goal pose (x, y, phi), phi is in Degree', required=True)
    parser.add_argument('-r', '--radius', type=float,
                        help='specify turning radius', default=1.0)
    args = parser.parse_args()

    rs = ReedsShepp(start=(args.start[0], args.start[1], args.start[2] * DEG),
                    goal=(args.goal[0], args.goal[1], args.goal[2] * DEG),
                    radius=args.radius)

    print("Driving from {} to {}".format(tuple(args.start), tuple(args.goal)))
    print("The optimal Reeds-Shepp type is {}, the minimum distance is {:.3f}(m)."
          .format([t.name for t in rs.path.type], rs.distance))

    rs.draw()
    plt.show()
