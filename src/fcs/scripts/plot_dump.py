"""
Plot a trajectory written by the Rust version (fcs-rs) with `lead --dump <path>`.

Usage: uv run python -m fcs.scripts.plot_dump <path>
"""

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from fcs.plot_functions import plot_3d
from fcs.type import ApproximateResult, TrajectoryResult

SUPPORTED_FORMAT_VERSION = 1


def load_dump(path: Path) -> tuple[TrajectoryResult, ApproximateResult]:
    data: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    version = data.get("format_version")
    if version != SUPPORTED_FORMAT_VERSION:
        msg = f"unsupported format_version: {version} (expected {SUPPORTED_FORMAT_VERSION})"
        raise ValueError(msg)

    trajectory = data["trajectory"]
    closest = data["closest"]
    return (
        TrajectoryResult(
            t=np.array(trajectory["t"]),
            projectile_traj=np.array(trajectory["projectile_traj"]),
            target_traj=np.array(trajectory["target_traj"]),
            approx_index=trajectory["approx_index"],
        ),
        ApproximateResult(
            t=closest["t"],
            distance=closest["distance"],
            target_pos=tuple(closest["target_pos"]),
            proj_pos=tuple(closest["proj_pos"]),
        ),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a trajectory written by `lead --dump` of the Rust version")
    parser.add_argument("path", type=Path, help="JSON file written by `lead --dump`")
    args = parser.parse_args()

    trajectory_result, approx_result = load_dump(args.path)
    plot_3d(trajectory_result, approx_result)


if __name__ == "__main__":
    main()
