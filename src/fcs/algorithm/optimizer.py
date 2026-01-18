import time
from collections.abc import Callable

import numpy as np
from scipy.optimize import minimize

from fcs.algorithm.approximate_decision import calc_quadratic_param, find_approximation
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.type import AtmosphereModel, InitCond, OptimizeResult, RuntimeConst, Vector3


def _create_step(
    runtime_const: RuntimeConst, t_pos: Vector3, t_dir: Vector3
) -> Callable[[tuple[float, float]], float]:
    am = AtmosphereModel()

    def step(phi_theta: tuple[float, float]) -> float:
        phi, theta = phi_theta
        ic = InitCond(theta=theta, phi=phi, t_pos=t_pos, t_dir=t_dir)
        res = solve(ic, runtime_const, am, FallEvent(ic))
        aprox_param = calc_quadratic_param(res)
        approx_result = find_approximation(aprox_param, ic.t_pos, ic.t_dir)
        return approx_result.distance

    return step


def _initial_phi_theta(t_pos: Vector3) -> tuple[float, float]:
    dx, dy, dz = t_pos
    r_xy = np.sqrt(dx**2 + dy**2)
    phi = np.arctan2(dy, dx)
    theta = np.arctan2(dz, r_xy)
    return phi, theta


def run(runtime_const: RuntimeConst, t_pos: Vector3, t_dir: Vector3) -> OptimizeResult:
    start = time.perf_counter()

    step = _create_step(runtime_const, t_pos, t_dir)
    initial_phi, initial_theta = _initial_phi_theta(t_pos)

    result = minimize(
        step,
        x0=np.array([initial_phi, initial_theta]),
        bounds=[(-np.pi, np.pi), (0, np.pi / 2)],
        method="L-BFGS-B",
        options={"disp": False},
    )

    optimized_phi, optimized_theta = result.x
    end = time.perf_counter()
    return OptimizeResult(optimized_phi, optimized_theta, float(result.fun), (end - start) * 1000)
