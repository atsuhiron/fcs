from collections.abc import Callable
from typing import Literal, NamedTuple

import numpy as np

type State = tuple[float, float, float, float, float, float]
type Vector3 = tuple[float, float, float]
type StateEquation = Callable[[float, State], State]


class AtmosphereModel(NamedTuple):
    std_density: float = 1.225  # kg/m^3
    std_temp: float = 288.15  # K
    lapse_rate: float = 0.0065  # K/m
    index: float = 5.256


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
    projectile: np.ndarray
    target: np.ndarray
    approx_distance: float
    approx_index: int
