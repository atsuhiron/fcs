from collections.abc import Iterable

from matplotlib import pyplot as plt

from fcs.type import TrajectoryResult


def plot_xz(trajectory_results: Iterable[TrajectoryResult], names: Iterable[str]) -> None:
    for trajectory_result, name in zip(trajectory_results, names, strict=True):
        x = trajectory_result.projectile[0]
        z = trajectory_result.projectile[2]
        plt.plot(x, z, "o", label=name, markersize=4)

    plt.legend()
    plt.xlabel("X axis")
    plt.ylabel("Z axis")
    plt.show()


def plot_3d(trajectory_result: TrajectoryResult) -> None:
    px, py, pz = trajectory_result.projectile
    tx, ty, tz = trajectory_result.target
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
