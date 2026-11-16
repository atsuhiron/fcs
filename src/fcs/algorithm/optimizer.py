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
INITIAL_STEP = 0.02  # rad
SEARCH_TOLERANCE = 1e-6


def _create_evaluator(
    runtime_const: RuntimeConst, t_pos: Vector3, t_dir: Vector3, atmosphere_model: AtmosphereModel
) -> Callable[[tuple[float, float]], tuple[ApproximateResult, bool]]:
    """Create a function that returns the closest point and whether the trajectory was cut off at span_end."""

    def evaluate(phi_theta: tuple[float, float]) -> tuple[ApproximateResult, bool]:
        phi, theta = phi_theta
        ic = InitCond(theta=theta, phi=phi, t_pos=t_pos, t_dir=t_dir)
        res = solve(ic, runtime_const, atmosphere_model, FallEvent(ic))
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
    atmosphere_model: AtmosphereModel | None = None,
    tolerance: float = 0.1,
) -> OptimizeResult:
    """
    Find the launch angles that bring the projectile closest to the target.

    The result is a "hit" if the closest distance is within `tolerance` (m).
    """
    start = time.perf_counter()

    if atmosphere_model is None:
        atmosphere_model = AtmosphereModel()
    evaluate = _create_evaluator(runtime_const, t_pos, t_dir, atmosphere_model)
    initial_phi, initial_theta = _initial_phi_theta(t_pos)

    def clip_theta(theta: float) -> float:
        # theta は天頂 (pi/2) を越えて反対側まで動けるようにする。天頂では phi を変えても弾道が変わらないため、
        # 上限を pi/2 にすると、phi が逆向きのまま天頂に達したときにそこから動けなくなる
        return min(max(float(theta), 0.0), float(np.pi))

    def squared_distance(phi_theta: np.ndarray) -> float:
        # 距離の2乗は最小点の近くでも滑らか
        return evaluate((float(phi_theta[0]), clip_theta(phi_theta[1])))[0].distance ** 2

    result = minimize(
        squared_distance,
        x0=np.array([initial_phi, initial_theta]),
        # phi は周期的なので制限せず、結果を (-pi, pi] に畳み込む。
        # theta の範囲は bounds ではなく目的関数の中で制限する。bounds を渡すと一次元探索が
        # 区間全体を対象にして、別の谷へ飛んでしまう
        method="Powell",
        options={
            # 解は初期値 (目標の現在位置の方向) から数度以内にあることが多い。既定の 1 rad では大きすぎて命中を逃す
            "direc": np.eye(2) * INITIAL_STEP,
            "xtol": SEARCH_TOLERANCE,
            "ftol": SEARCH_TOLERANCE,
            # 命中する場合はほぼ 300 回未満で収束する。届かない目標で探索が長引くのを防ぐ
            "maxfev": MAX_EVALUATIONS,
        },
    )

    # result.success は命中かどうかと対応しない (評価回数の上限に達しても命中していることがあり、
    # 届かないまま「収束」することもある) ので、最接近距離そのもので判定する
    optimized_phi = float(result.x[0])
    optimized_theta = clip_theta(result.x[1])
    if optimized_theta > np.pi / 2:
        # 天頂を越えた向きは、方位角を反転して仰角を折り返した向きと同じ
        optimized_phi += np.pi
        optimized_theta = np.pi - optimized_theta
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
