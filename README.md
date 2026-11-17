# FCS
Based on the target's position and velocity information,
it calculates the aiming data required to make the projectile collide with the target.

> [!NOTE]
> A Rust version, [fcs-rs](https://github.com/atsuhiron/fcs-rs), runs the same algorithm as a single binary
> and is over 100 times faster. Use it for the calculation itself.
> This repository is the reference implementation: it holds the derivation of the mathematical model (below),
> generates the reference data that fcs-rs is tested against, and does the plotting.
> It is frozen and changes only when the algorithm itself changes.

## Input Data
The specifications for the projectile are as follows:
- Initial velocity ($V_0$)
- Mass ($m$)
- Forward-projecting area ($A$)
- Drag coefficient ($C_D$)
- Type of drag ($I$): $I = 1$ for pressure drag, proportional to the square of the velocity (`--drag-type pressure`, default), or $I = 0$ for friction drag, proportional to the velocity (`--drag-type friction`)

The target specifications are as follows:
- position ($\vec{r_0}$)
- Velocity (${\vec{d}}$)

Additionally, the parameters related to air resistance are as follows:
- Standard density ($D_0 = 1.225\ {\rm kg/m^3}$)
- Standard temperature ($T_0 = 288.15\ {\rm K}$)
- Lapse rate ($L = 0.0065\ {\rm K/m}$)
- Exponent ($h_0 = 5.256$)

Air density is expressed according to the International Standard Atmosphere (ISA) model 
using the following equation:
```math
D(z) = D_0 \left( 1 - \frac{Lz}{T_0} \right) ^{h_0}
```
When initial velocity and mass are kept constant, the trajectory changes as shown in the following graph when multiple conditions are altered.
![trajectories](images/trajectory_sample.png)

## Output Data
Two angles, $(\phi, \theta)$ representing the aiming direction are output.
- Azimuth $(\phi: -\pi \leq \phi \leq \pi)$
- Elevation $(\theta: 0 \leq \theta \leq \pi/2)$
- Distance
- ETA time (msec)
- Calculation time (msec)

## Installation
If you can use `uv`, you can install by following command:
```shell
$ uv sync
```

## Usage
If you can use `uv`, you can run following command:
```shell
$ uv run lead 0.1 0.001 0.1 300 100 100 100 -10 1 0
46.95 36.46 0.00 601.31 158.00
``` 
or not:
```shell
$ python .\src\fcs\main.py 0.1 0.001 0.1 300 100 100 100 -10 1 0
46.95 36.46 0.00 601.31 158.00
``` 
Adding the `-r` (`--readable`) option will output the results in a more human-readable format.
```shell
$ lead -r 0.1 0.001 0.1 300 100 100 100 -10 1 0
phi: 46.95°, theta: 36.46°, dist: 0.00, eta_time: 601.31 ms, calc_time: 158.00 ms
``` 
Adding the `--plot` option will show trajectory of projectile and target.
```shell
$ lead --plot 0.1 0.001 0.1 300 100 100 100 -10 1 0
46.95 36.46 0.00 601.31 158.00
``` 
![calculated_trajectory](images/trajectory_3d.png)

### With the Rust version
The `lead` command of [fcs-rs](https://github.com/atsuhiron/fcs-rs) takes the same arguments and gives the same output and exit statuses.
It has no `--plot`; instead it writes the trajectory to a file with `--dump`, which this repository can plot:
```shell
$ lead --dump trajectory.json 0.1 0.001 0.1 300 100 100 100 -10 1 0
46.95 36.46 0.00 601.31 1.24
$ uv run python -m fcs.scripts.plot_dump trajectory.json
```
The reference data for the tests of fcs-rs is generated here (it takes about a minute):
```shell
$ uv run python -m fcs.scripts.dump_fixtures <path to fcs-rs>/tests/fixtures
```

### Exit status
The result counts as a hit when the closest distance between the projectile and the target is within `--tolerance` (default: 0.1 m).
The exit status tells whether the projectile hits the target, and why not if it does not.

| Exit status | Name | Meaning |
|---|---|---|
| 0 | `hit` | The projectile hits the target. |
| 1 | | Unexpected error. |
| 2 | | Invalid arguments. The arguments are checked before the calculation starts (see below). |
| 3 | `out_of_range` | The projectile lands without reaching the target. |
| 4 | `out_of_time` | The projectile is still in flight at `--span-end` (default: 10 s) and has not reached the target. A larger `--span-end` may give a hit. |

For 3 and 4, the result for the closest approach is still printed to stdout, and `lead: <name>: <message>` is written to stderr.
```shell
$ lead 0.1 0.001 0.1 300 4000 0 500 0 50 0
7.13 14.48 2403.53 10000.00 208.00
lead: out_of_time: the projectile does not reach the target within --span-end
```
The following arguments are rejected with exit status 2, and nothing is printed to stdout:
- Any value that is not a finite number (`nan`, `inf`)
- `mass`, `area`, `initial_velocity`, `--span-end` or `--std-temp` that is 0 or less
- `drag_coefficient`, `target_z`, `--tolerance`, `--std-density` or `--lapse-rate` that is negative
- A target at the launch point `(0, 0, 0)`
```shell
$ lead 0.1 0.001 0.1 300 100 100 -5 0 0 0
usage: lead [-h] ...
lead: error: target_z must be 0 or greater: -5.0
```

## Procedure
Follow these steps to aim.
1. Set initial angles ($\phi_0, \theta_0$)
2. Calculate the trajectory based on two angle measurements and the input data.
3. Calculate the closest distance between the projectile and the target.
4. Optimize the two angles to minimize this distance (Repeat steps 2 and 3).
   
### 1. Set initial angles
Any method that allows for rough targeting is acceptable. Here, the angle is determined using the target's current coordinates.
```math
\begin{align}
\phi_0 &= \arctan \left( \frac{r_{0y}}{r_{0x}} \right)\\
\theta_0 &= \arctan \left( \frac{r_{0z}}{\sqrt{r_{0x}^2 + r_{0y}^2}} \right)
\end{align}
```

### 2. Calculate the trajectory
Solving the following equation will reveal the $t_{\rm apx}$ when the projectile and target are closest to each other.
```math
\begin{align}
\vec{r_0} + t\vec{d} = t\vec{V_0} - \vec{F}
\end{align}
```
where $\vec{F}$ represents the vector of the total force acting on the projectile. The left side represents the target's predicted trajectory, while the right side represents the projectile's trajectory. If forces other than gravity are ignored, or if air resistance (frictional resistance) is proportional to velocity, this equation can be solved analytically. However, in reality, due to various effects, $\vec{F}$ takes on a complex form and cannot be solved analytically.  
Therefore, numerically solve the following system of first-order ordinary differential equations:
```math
\begin{align}
\frac{d\vec{S}}{dt} = \frac{d}{dt}
\begin{pmatrix}
x \\
y \\
z \\
v_x \\
v_y \\
v_z
\end{pmatrix} = 
\begin{pmatrix}
v_x \\
v_y \\
v_z \\
-C(v_x + d_x) \\
-C(v_y + d_y) \\
-C(v_z + d_z) - g
\end{pmatrix}
\end{align}
```
Here $\vec{S}$ is the state of the projectile relative to the target. Air resistance, however, depends on the motion relative to the air, so it is evaluated with the absolute velocity $\vec{v} + \vec{d}$ and the absolute altitude $z + r_{0z} + d_z t$ of the projectile. $C$ is the air resistance coefficient and follows the following equation.
```math
C = \frac{AC_DD(z + r_{0z} + d_z t)|\vec{v} + \vec{d}|^{I}}{2m}
```
Use the following values as the initial conditions for the differential equation:
```math
\begin{align}
\vec{S}_0 = 
\begin{pmatrix}
-r_{0x} \\
-r_{0y} \\
-r_{0z} \\
-d_x + V_0\cos{\theta_0}\cos{\phi_0} \\
-d_y + V_0\cos{\theta_0}\sin{\phi_0} \\
-d_z + V_0\sin{\theta_0}
\end{pmatrix}
\end{align}
```

### 3. Calculate the closest point
Solving the above differential equation yields the trajectory of the projectile. However, this is a discrete set of points $\vec{B}_n$ obtained through numerical analysis, and its minimum value $\tilde{\delta}$ is not the true closest distance. Since the trajectory of the projectile cannot be expressed analytically, to find the true closest distance $\delta$, it is necessary to approximate the trajectory near $\tilde{\delta}$ using an analytical function. Here, we will approximate it using a quadratic function.
For simplicity, we assume the trajectory of the projectile $\vec{B'}(t)$ lies entirely within the $x'z$ plane. In this case, the transformation equations between the $xyz$ coordinate system and the $x'y'z'$ coordinate system are as follows.
```math
\begin{align}
\begin{pmatrix}
x' \\
y' \\
z'
\end{pmatrix} &= R_z(-\tilde{\phi})
\begin{pmatrix}
x \\
y \\
z
\end{pmatrix}\\
&= \begin{pmatrix}
\cos\tilde{\phi} & \sin\tilde{\phi} & 0\\
-\sin\tilde{\phi} & \cos\tilde{\phi} & 0\\
0 & 0 & 1
\end{pmatrix}
\begin{pmatrix}
x \\
y \\
z
\end{pmatrix}
\end{align}
```
where $R_z(\tilde{\phi})$ is the rotation matrix about the $z$-axis, and $\tilde{\phi}$ is expressed using $\vec{B}_n$, the result of numerical computation, in the following equation.
```math
\tilde{\phi} = \arctan \left( \frac{\sum_{i} B_y(\tilde{t}_i)}{\sum_{i} B_x(\tilde{t}_i)} \right)
```
The sum is taken over the points used for the approximation. This is the direction of the sum of their position vectors, so the launch point (the origin, whose azimuth is undefined) has no effect, and the result is not affected by the wrap-around of the angle at $\pm\pi$.
In this $x'y'z'$ coordinate system, we approximate the trajectory near $\tilde{t}_n$ as follows:
```math
\begin{align}
\vec{B'}(t) =
\begin{pmatrix}
x'(t) \\
y'(t) \\
z'(t)
\end{pmatrix}
\approx 
\begin{pmatrix}
a_xt^2 + b_xt + c_x\\
0 \\
a_zt^2 + b_zt + c_z
\end{pmatrix}
\end{align}
```
Similarly, if we denote the predicted trajectory $\vec{T}$ of the target in the $x'y'z'$ coordinate system as $\vec{T'}$, the problem of finding $\delta$ reduces to the following minimization problem.
```math
\begin{align}
\delta &= \min_t \left| \vec{B'}(t) - \vec{T'}(t)\right| \\

&= \min_t \left|
\begin{pmatrix}
x'(t) \\
y'(t) \\
z'(t)
\end{pmatrix}
- R_z(-\tilde{\phi})\left[ t\vec{d} + \vec{r}_0\right]\right| \\

&= \min_t \left|
\begin{pmatrix}
a_xt^2 + b_xt + c_x\\
0 \\
a_zt^2 + b_zt + c_z
\end{pmatrix} - R_z(-\tilde{\phi})\left[
\begin{pmatrix}
d_x \\
d_y \\
d_z
\end{pmatrix}
t + 
\begin{pmatrix}
r_{0x} \\
r_{0y} \\
r_{0z}
\end{pmatrix}
\right]\right| \\

&= \min_t \left|
\begin{array}{rrr}
a_x t^2 & +\left( b_x - d_x \cos \tilde{\phi} - d_y \sin \tilde{\phi} \right)t & +(c_x - r_{0x} \cos \tilde{\phi} - r_{0y} \sin \tilde{\phi}) \\
& \left(d_x \sin \tilde{\phi} - d_y \cos \tilde{\phi} \right)t & +(r_{0x} \sin \tilde{\phi} - r_{0y} \cos \tilde{\phi}) \\
a_z t^2 & + \left(b_z - d_z \right)t & + \left(c_z - r_{0z}\right)
\end{array}
\right|
\end{align}
```
Since $a_x^2 + a_z^2 > 0$, this is equivalent to finding the minimum value of a quartic function. The quadratic approximation is valid only near $\tilde{t}_n$, so the minimum is searched within the time range of the points used for the approximation. If the trajectory has fewer than three points (for example, the projectile lands immediately after launch), the approximation is skipped and $\tilde{\delta}$ is used as is.

### 4. Optimize the two angles
In simple terms, what we are doing in steps 2 and 3 is determining the closest distance $\delta$ when a projectile is launched at two angles, $\phi$ and $\theta$. Therefore, Eq. (9) above can be rewritten by adding phi and theta as arguments as follows.
```math
\begin{align}
\delta(\phi, \theta) &= \min_t \left| \vec{B'}(t, \phi, \theta) - \vec{T'}(t)\right|
\end{align}
```
Ultimately, we seek the launch angles $\phi_{\rm apx}$ and $\theta_{\rm apx}$ that minimize the distance $\delta$ between the projectile and the target, which reduces to the following minimization problem.
```math
\begin{align}
\delta_{\rm apx} &= \delta(\phi_{\rm apx}, \theta_{\rm apx})\\
&= \min_{\phi, \theta} \delta(\phi, \theta)\\
&= \min_{\phi, \theta} \min_t \left| \vec{B'}(t, \phi, \theta) - \vec{T'}(t)\right|
\end{align}
```
The search range of $\theta$ is $0 \leq \theta \leq \pi$, not $0 \leq \theta \leq \pi/2$. At the zenith ($\theta = \pi/2$), $\phi$ has no effect on the trajectory, so a search that reaches the zenith with $\phi$ pointing away from the target cannot leave it. Allowing $\theta$ to pass through the zenith removes this. Since the direction $(\phi, \theta)$ with $\theta > \pi/2$ is the same as $(\phi + \pi, \pi - \theta)$, the result is converted back to $0 \leq \theta \leq \pi/2$.
