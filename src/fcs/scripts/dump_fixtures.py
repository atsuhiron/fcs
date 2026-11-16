"""
Dump reference data for the Rust port (fcs-rs).

Usage: uv run python -m fcs.scripts.dump_fixtures <output_dir>

Two files are written.
- trajectories.json: trajectories (layer 1) and the closest point found on each (layer 2).
- optimizations.json: results of the aim search (layer 3), some with every evaluated point.
"""

import argparse
import json
import subprocess
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import Any

import numpy as np
import scipy

import fcs.algorithm.optimizer as optimizer_module
from fcs.algorithm.approximate_decision import find_closest_point
from fcs.algorithm.optimizer import INITIAL_STEP, MAX_EVALUATIONS, SEARCH_TOLERANCE, run_optimization
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.type import ApproximateResult, AtmosphereModel, InitCond, RuntimeConst, Vector3

type Target = tuple[Vector3, Vector3]
type Evaluator = Callable[[tuple[float, float]], tuple[ApproximateResult, bool]]

TOLERANCE = 0.1
TRACED_PER_SET = 3  # 各集合の先頭の何件について、評価した点を全て記録するか

STANDARD = RuntimeConst(m=0.1, area=1e-3, cd=0.1, v0=300, span_end=10)
FAST = RuntimeConst(m=0.01, area=1e-4, cd=0.3, v0=800, span_end=10)
SLOW_FRICTION = RuntimeConst(m=0.1, area=1e-3, cd=0.1, v0=100, span_end=20, drag_type="friction")
SHORT_SPAN = RuntimeConst(m=0.1, area=1e-3, cd=0.1, v0=300, span_end=2)
NO_DRAG = RuntimeConst(m=0.1, area=1e-3, cd=0.0, v0=300, span_end=100)
THIN_AIR = AtmosphereModel(std_density=0.9, std_temp=250.0, lapse_rate=0.005, index_=4.5)


def _meta() -> dict[str, Any]:
    def git(*args: str) -> str:
        return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout.strip()  # noqa: S603, S607

    return {
        "python_commit": git("rev-parse", "HEAD"),
        "python_worktree_dirty": bool(git("status", "--porcelain")),
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "search": {
            "method": "Powell",
            "initial_step": INITIAL_STEP,
            "tolerance": SEARCH_TOLERANCE,
            "max_evaluations": MAX_EVALUATIONS,
        },
    }


def _random_targets(  # noqa: PLR0913
    seed: int,
    count: int,
    r_range: tuple[float, float],
    z_range: tuple[float, float],
    v_xy: float,
    v_z: float,
) -> list[Target]:
    rng = np.random.default_rng(seed)
    targets: list[Target] = []
    for _ in range(count):
        r, azimuth = float(rng.uniform(*r_range)), float(rng.uniform(-np.pi, np.pi))
        z = float(rng.uniform(*z_range))
        vx, vy = (float(v) for v in rng.uniform(-v_xy, v_xy, 2))
        vz = float(rng.uniform(-v_z, v_z))
        targets.append(((r * np.cos(azimuth), r * np.sin(azimuth), z), (vx, vy, vz)))
    return targets


def _dump_trajectories() -> list[dict[str, Any]]:
    targets: list[Target] = [
        ((100.0, 100.0, 100.0), (-10.0, 1.0, 0.0)),
        ((-400.0, 250.0, 300.0), (30.0, -20.0, 4.0)),
        ((50.0, -600.0, 0.0), (0.0, 0.0, 0.0)),
    ]
    theta_deg = [0.0, 1e-4, 0.5, 5.0, 20.0, 36.46, 45.0, 70.0, 89.0, 90.0, 120.0]
    phi_deg = [46.95, -150.0]
    models: list[tuple[str, RuntimeConst, AtmosphereModel]] = [
        ("standard", STANDARD, AtmosphereModel()),
        ("fast", FAST, AtmosphereModel()),
        ("slow_friction", SLOW_FRICTION, AtmosphereModel()),
        ("short_span", SHORT_SPAN, AtmosphereModel()),
        ("no_drag", NO_DRAG, AtmosphereModel()),
        ("thin_air", STANDARD, THIN_AIR),
    ]

    cases: list[dict[str, Any]] = []
    for name, runtime_const, atmosphere in models:
        for t_pos, t_dir in targets:
            for theta in theta_deg:
                for phi in phi_deg:
                    ic = InitCond(theta=float(np.deg2rad(theta)), phi=float(np.deg2rad(phi)), t_pos=t_pos, t_dir=t_dir)
                    res = solve(ic, runtime_const, atmosphere, FallEvent(ic))
                    closest = find_closest_point(res, t_pos, t_dir)
                    cases.append(
                        {
                            "model": name,
                            "runtime_const": runtime_const._asdict(),
                            "atmosphere": atmosphere._asdict(),
                            "theta": ic.theta,
                            "phi": ic.phi,
                            "t_pos": t_pos,
                            "t_dir": t_dir,
                            "t": res.t.tolist(),
                            "projectile": res.projectile_traj.tolist(),
                            "target": res.target_traj.tolist(),
                            "approx_index": res.approx_index,
                            "closest": closest._asdict(),
                        }
                    )
    return cases


@contextmanager
def _recording_evaluations(trace: list[tuple[float, float, float]]) -> Iterator[None]:
    """Record every (phi, theta, distance) that the optimizer evaluates, in order."""
    original = optimizer_module._create_evaluator  # noqa: SLF001

    def create(*args: Any) -> Evaluator:  # noqa: ANN401
        evaluate = original(*args)

        def recording(phi_theta: tuple[float, float]) -> tuple[ApproximateResult, bool]:
            result = evaluate(phi_theta)
            trace.append((float(phi_theta[0]), float(phi_theta[1]), result[0].distance))
            return result

        return recording

    optimizer_module._create_evaluator = create  # noqa: SLF001  # ty: ignore[invalid-assignment]
    try:
        yield
    finally:
        optimizer_module._create_evaluator = original  # noqa: SLF001


def _dump_optimizations() -> list[dict[str, Any]]:
    sets: list[tuple[str, RuntimeConst, list[Target]]] = [
        ("readme", STANDARD, [((100.0, 100.0, 100.0), (-10.0, 1.0, 0.0)), ((4000.0, 0.0, 500.0), (0.0, 50.0, 0.0))]),
        ("near", STANDARD, _random_targets(1, 100, (50, 1200), (20, 800), 60, 5)),
        ("overhead", STANDARD, _random_targets(11, 40, (0, 4), (100, 500), 6, 0)),
        ("ground", STANDARD, _random_targets(7, 40, (50, 800), (0, 0), 30, 0)),
        ("far", STANDARD, _random_targets(2, 20, (2500, 6000), (20, 800), 60, 5)),
        ("fast", FAST, _random_targets(3, 40, (50, 1500), (0, 600), 80, 10)),
        ("slow_friction", SLOW_FRICTION, _random_targets(4, 40, (20, 400), (0, 150), 15, 2)),
    ]
    atmosphere = AtmosphereModel()

    cases: list[dict[str, Any]] = []
    for name, runtime_const, targets in sets:
        for index, (t_pos, t_dir) in enumerate(targets):
            trace: list[tuple[float, float, float]] = []
            with _recording_evaluations(trace):
                result = run_optimization(runtime_const, t_pos, t_dir, atmosphere, TOLERANCE)
            case: dict[str, Any] = {
                "set": name,
                "runtime_const": runtime_const._asdict(),
                "atmosphere": atmosphere._asdict(),
                "t_pos": t_pos,
                "t_dir": t_dir,
                "tolerance": TOLERANCE,
                "status": result.status,
                "phi": result.phi,
                "theta": result.theta,
                "distance": result.distance,
                "eta_msec": result.eta_msec,
                "format_cl": result.format_cl().rsplit(" ", 1)[0],  # 計算時間の列を除く
                # 探索中の評価に、探索後の確認の 1 回を加えた数
                "evaluations": len(trace),
            }
            if index < TRACED_PER_SET:
                case["trace"] = trace
            cases.append(case)
    return cases


def main() -> None:
    parser = argparse.ArgumentParser(description="Dump reference data for the Rust port")
    parser.add_argument("output_dir", type=Path)
    output_dir: Path = parser.parse_args().output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    meta = _meta()
    for file_name, cases in (
        ("trajectories.json", _dump_trajectories()),
        ("optimizations.json", _dump_optimizations()),
    ):
        path = output_dir / file_name
        path.write_text(json.dumps({"meta": meta, "cases": cases}), encoding="utf-8", newline="\n")
        print(f"{path}: {len(cases)} cases")  # noqa: T201


if __name__ == "__main__":
    main()
