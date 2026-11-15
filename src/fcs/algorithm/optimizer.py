import time
from collections.abc import Callable

import numpy as np
from scipy.optimize import minimize

from fcs.algorithm.approximate_decision import find_closest_point
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.type import (
    ApproximateResult,
    AtmosphereModel,
    InitCond,
    OptimizeResult,
    OptimizeStatus,
    RuntimeConst,
    Vector3,
)

MAX_EVALUATIONS = 300


def _create_evaluator(
    runtime_const: RuntimeConst, t_pos: Vector3, t_dir: Vector3, atoms_model: AtmosphereModel
) -> Callable[[tuple[float, float]], tuple[ApproximateResult, bool]]:
    """Create a function that returns the closest point and whether the trajectory was cut off at span_end."""

    def evaluate(phi_theta: tuple[float, float]) -> tuple[ApproximateResult, bool]:
        phi, theta = phi_theta
        ic = InitCond(theta=theta, phi=phi, t_pos=t_pos, t_dir=t_dir)
        res = solve(ic, runtime_const, atoms_model, FallEvent(ic))
        # 着地せずに終わった弾道は、最後の時刻がちょうど span_end になる
        cut_off = bool(res.t[-1] >= runtime_const.span_end)
        return find_closest_point(res, ic.t_pos, ic.t_dir), cut_off

    return evaluate


def _wrap_angle(phi: float) -> float:
    """Wrap an angle into (-pi, pi]."""
    return float(np.pi - (np.pi - phi) % (2 * np.pi))


def _initial_phi_theta(t_pos: Vector3) -> tuple[float, float]:
    dx, dy, dz = t_pos
    r_xy = np.sqrt(dx**2 + dy**2)
    phi = np.arctan2(dy, dx)
    theta = np.arctan2(dz, r_xy)
    return phi, theta


def run_optimization(
    runtime_const: RuntimeConst,
    t_pos: Vector3,
    t_dir: Vector3,
    atoms_model: AtmosphereModel | None = None,
    tolerance: float = 0.1,
) -> OptimizeResult:
    """
    Find the launch angles that bring the projectile closest to the target.

    The result is a "hit" if the closest distance is within `tolerance` (m).
    """
    start = time.perf_counter()

    if atoms_model is None:
        atoms_model = AtmosphereModel()
    evaluate = _create_evaluator(runtime_const, t_pos, t_dir, atoms_model)
    initial_phi, initial_theta = _initial_phi_theta(t_pos)

    result = minimize(
        lambda phi_theta: evaluate(phi_theta)[0].distance ** 2,  # 距離の2乗は最小点の近くでも滑らか
        x0=np.array([initial_phi, initial_theta]),
        bounds=[(None, None), (0, np.pi / 2)],  # phi は周期的なので制限せず、結果を (-pi, pi] に畳み込む
        method="L-BFGS-B",
        # 命中する場合は 200 回未満で収束する。届かない目標で探索が長引くのを防ぐ
        options={"maxfun": MAX_EVALUATIONS},
    )

    # result.success は命中かどうかと対応しない (命中後に直線探索が異常終了する、
    # 届かないまま「収束」する、のどちらもある) ので、最接近距離そのもので判定する
    optimized_phi, optimized_theta = (float(x) for x in result.x)
    approx_result, cut_off = evaluate((optimized_phi, optimized_theta))
    status: OptimizeStatus
    if approx_result.distance <= tolerance:
        status = "hit"
    elif cut_off:
        status = "out_of_time"
    else:
        status = "out_of_range"

    end = time.perf_counter()
    return OptimizeResult(
        status=status,
        phi=_wrap_angle(optimized_phi),
        theta=optimized_theta,
        distance=approx_result.distance,
        eta_msec=approx_result.t * 1000,
        calc_msec=(end - start) * 1000,
    )
