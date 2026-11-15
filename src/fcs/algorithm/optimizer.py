import time
from collections.abc import Callable

import numpy as np
from scipy.optimize import minimize

from fcs.algorithm.approximate_decision import find_closest_point
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.type import ApproximateResult, AtmosphereModel, InitCond, OptimizeResult, RuntimeConst, Vector3


def _create_evaluator(
    runtime_const: RuntimeConst, t_pos: Vector3, t_dir: Vector3, atoms_model: AtmosphereModel
) -> Callable[[tuple[float, float]], ApproximateResult]:
    def evaluate(phi_theta: tuple[float, float]) -> ApproximateResult:
        phi, theta = phi_theta
        ic = InitCond(theta=theta, phi=phi, t_pos=t_pos, t_dir=t_dir)
        res = solve(ic, runtime_const, atoms_model, FallEvent(ic))
        return find_closest_point(res, ic.t_pos, ic.t_dir)

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
) -> OptimizeResult:
    start = time.perf_counter()

    if atoms_model is None:
        atoms_model = AtmosphereModel()
    evaluate = _create_evaluator(runtime_const, t_pos, t_dir, atoms_model)
    initial_phi, initial_theta = _initial_phi_theta(t_pos)

    result = minimize(
        lambda phi_theta: evaluate(phi_theta).distance ** 2,  # 距離の2乗は最小点の近くでも滑らか
        x0=np.array([initial_phi, initial_theta]),
        bounds=[(None, None), (0, np.pi / 2)],  # phi は周期的なので制限せず、結果を (-pi, pi] に畳み込む
        method="L-BFGS-B",
    )

    optimized_phi, optimized_theta = (float(x) for x in result.x)
    approx_result = evaluate((optimized_phi, optimized_theta))
    end = time.perf_counter()
    return OptimizeResult(
        phi=_wrap_angle(optimized_phi),
        theta=optimized_theta,
        distance=approx_result.distance,
        eta_msec=approx_result.t * 1000,
        calc_msec=(end - start) * 1000,
    )
