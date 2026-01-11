from collections.abc import Sequence

import numpy as np
from matplotlib import pyplot as plt

from fcs.algorithm.approximate_decision import rotate_xy
from fcs.type import QuadraticParam, TrajectoryResult


def plot_xz(trajectory_results: Sequence[TrajectoryResult], names: Sequence[str] | None = None) -> None:
    for trajectory_result, name in zip(trajectory_results, names or [None] * len(trajectory_results), strict=True):
        x = trajectory_result.projectile_traj[0]
        z = trajectory_result.projectile_traj[2]
        plt.plot(x, z, "o", label=name, markersize=4)

    plt.legend()
    plt.xlabel("X axis")
    plt.ylabel("Z axis")
    plt.show()


def plot_rotated_xz(trajectory_result: TrajectoryResult, q: QuadraticParam) -> None:
    x = trajectory_result.projectile_traj[0]
    y = trajectory_result.projectile_traj[1]
    z = trajectory_result.projectile_traj[2]

    # plot trajectory
    x_rotated, _ = rotate_xy(x, y, -q.phi)
    plt.plot(x_rotated, z, "o", label="projectile", markersize=4)

    # plot quadratic approximation
    x_fit = np.linspace(q.rx_dmain[0], q.rx_dmain[1], 100)
    z_fit = q.a * x_fit**2 + q.b * x_fit + q.c
    plt.plot(x_fit, z_fit, "-", label="approximation", color="red", lw=2)

    plt.title(f"Rotation angle φ = {np.rad2deg(q.phi):.2f}° ± {np.rad2deg(q.phi_std):.2f}°, RMSE = {q.rmse:.4f}")
    plt.legend()
    plt.xlabel("X' axis")
    plt.ylabel("Z axis")
    plt.show()


def plot_3d(trajectory_result: TrajectoryResult) -> None:
    px, py, pz = trajectory_result.projectile_traj
    tx, ty, tz = trajectory_result.target_traj
    approx_index = trajectory_result.approx_index

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(px, py, pz, label="projectile", color="blue", lw=2)
    ax.plot(tx, ty, tz, label="target", color="red")

    ax.scatter(px[approx_index], py[approx_index], pz[approx_index], label="approx", s=50)

    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.set_zlabel("Z axis")
    ax.legend()

    plt.show()
