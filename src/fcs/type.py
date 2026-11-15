from collections.abc import Callable
from typing import Literal, NamedTuple

import numpy as np

type State = tuple[float, float, float, float, float, float]
type Vector3 = tuple[float, float, float]
type StateEquation = Callable[[float, State], State]


# Solving ODE


class AtmosphereModel(NamedTuple):
    std_density: float = 1.225  # kg/m^3
    std_temp: float = 288.15  # K
    lapse_rate: float = 0.0065  # K/m
    index_: float = 5.256


class InitCond(NamedTuple):
    theta: float
    phi: float
    t_pos: Vector3
    t_dir: Vector3


class RuntimeConst(NamedTuple):
    m: float
    area: float
    cd: float
    v0: float
    span_end: float  # 発射 (t = 0) から計算を打ち切るまでの時間 (s)
    drag_type: Literal["pressure", "friction"] = "pressure"


class TrajectoryResult(NamedTuple):
    t: np.ndarray
    projectile_traj: np.ndarray
    target_traj: np.ndarray
    approx_index: int


# Approximate Decision


class QuadraticParam(NamedTuple):
    a: float
    b: float
    c: float
    domain: tuple[float, float]
    rmse: float


class ApproximateParam(NamedTuple):
    x_param: QuadraticParam
    z_param: QuadraticParam
    phi: float
    phi_std: float


class ApproximateResult(NamedTuple):
    t: float
    distance: float
    target_pos: Vector3
    proj_pos: Vector3


# "hit" は最接近距離が許容値以下の場合。
# "out_of_time" は許容値を超えていて、弾が着地する前に計算が span_end で打ち切られた場合。
# span_end を延ばせば届く可能性がある。
# "out_of_range" は許容値を超えていて、弾が span_end より前に着地した場合。この弾では届かない。
type OptimizeStatus = Literal["hit", "out_of_time", "out_of_range"]


class OptimizeResult(NamedTuple):
    phi: float
    theta: float
    distance: float
    eta_msec: float
    calc_msec: float
    status: OptimizeStatus

    def format_cl(self) -> str:
        return (
            f"{np.rad2deg(self.phi):.2f} {np.rad2deg(self.theta):.2f} "
            f"{self.distance:.2f} {self.eta_msec:.2f} {self.calc_msec:.2f}"
        )

    def format_h(self) -> str:
        return (
            f"phi: {np.rad2deg(self.phi):.2f}°, "
            f"theta: {np.rad2deg(self.theta):.2f}°, "
            f"dist: {self.distance:.2f}, "
            f"eta_time: {self.eta_msec:.2f} ms, "
            f"calc_time: {self.calc_msec:.2f} ms"
        )
