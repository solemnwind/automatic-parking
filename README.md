# Automatic Parking

This project implements an algorithm for automatic parking, 
specifically designed for four-wheeled vehicles with front-wheel steering.

## Simulation Configuration

### Vehicle Model

The vehicle is modeled with [Ackermann steering geometry](https://en.wikipedia.org/wiki/Ackermann_steering_geometry), 
the origin $`O`$ is chosen to be the center of the rear axle,
because the center of the turning circle always lies on the extension of the rear axle. 

<p align="center"><img src="resources/modeling.png" width=400></p>

<!-- ### Kinematic Equations of The Vehicle

```math
\left\{\begin{align*}
\dot{x} &= v \cos(\varphi)\\
\dot{y} &= v \sin(\varphi)\\
\dot{v} &= a\\
\dot{\varphi} &= \frac{v}{L}\tan(\theta)\\
\dot{\theta} &= \omega
\end{align*}\right.
```

The changing rate of the turning angle $`\left|\omega\right| < 1 \ \mathrm{rad\!\cdot\! s^{-1}}`$. -->

### Simulation Environment

The configuration file ([example](./src/utils/test_parking_lot.toml)) defines a test scene including obstacles, one parking slot and one vehicle.

## Usage

To get the shortest Reeds-Shepp curve from one point to another, run:
```bash
python reeds_shepp.py [-h] [-s X Y PHI] -e X Y PHI [-r RADIUS]
```

To run the simulation:
```bash
python ./src/simulations/simulator.py
```
You can change the scene setup and configurations in `src/utils/test_parking_lot.toml`
or create a new configuration file.


## Demos

### Path finding

<p float="left">
<img src="resources/parking-lot-1.png" width=400>
<img src="resources/parking-lot-1.gif" width=400>
</p>

<p float="left">
<img src="resources/parking-lot-2.png" width=400>
<img src="resources/parking-lot-2.gif" width=400>
</p>

<p float="left">
<img src="resources/parking-lot-3.png" width=400>
<img src="resources/parking-lot-3.gif" width=400>
</p>

<p float="left">
<img src="resources/parking-lot-4.png" width=400>
<img src="resources/parking-lot-4.gif" width=400>
</p>

<p float="left">
<img src="resources/parking-lot-5.png" width=400>
<img src="resources/parking-lot-5.gif" width=400>
</p>

### Reeds-Shepp curve

<p float="left">
<img src="resources/reeds-shepp-1.png" width=400>
<img src="resources/reeds-shepp-2.png" width=400>
<img src="resources/reeds-shepp-3.png" width=400>
<img src="resources/reeds-shepp-4.png" width=400>
</p>
