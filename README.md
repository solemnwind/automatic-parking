# Automatic Parking

This project implements an algorithm for automatic parking, 
specifically designed for four-wheeled vehicles with front-wheel steering.

## Kinematics

### Modeling

The vehicle is modeled with [Ackermann steering geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry), 
the origin $O$ is chosen to be the center of the rear axle,
because the center of the turning circle always lies on the extension of the rear axle. 

![Ackermann geometry](resources/modeling.png)

### Kinematic Equations

$$
\left\{\begin{align*}
\dot{x} &= v \cos(\varphi)\\
\dot{y} &= v \sin(\varphi)\\
\dot{v} &= a\\
\dot{\varphi} &= \frac{v}{L}\tan(\theta)\\
\dot{\theta} &= \omega
\end{align*}\right.
$$

The changing rate of the turning angle $\left|\omega\right| < 1 \,\,\mathrm{rad\cdot s^{-1}}$.
