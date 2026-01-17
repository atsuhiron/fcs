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
    span: tuple[float, float]
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


class OptimizeResult(NamedTuple):
    phi: float
    theta: float
    msec: float

    def __str__(self) -> str:
        return f"phi: {np.rad2deg(self.phi):.2f}°, theta: {np.rad2deg(self.theta):.2f}°, time: {self.msec:.2f} ms"
