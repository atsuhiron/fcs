import numpy as np
from scipy.constants import g
from scipy.integrate import solve_ivp

from fcs.algorithm.terminal_events import TerminalEvent
from fcs.type import AtmosphereModel, InitCond, RuntimeConst, State, StateEquation, TrajectoryResult
from fcs.util import reconstruct_target_trajectory


def create_init_state(init_cond: InitCond, runtime_const: RuntimeConst) -> State:
    return (
        -init_cond.t_pos[0],
        -init_cond.t_pos[1],
        -init_cond.t_pos[2],
        runtime_const.v0 * np.cos(init_cond.theta) * np.cos(init_cond.phi) - init_cond.t_dir[0],
        runtime_const.v0 * np.cos(init_cond.theta) * np.sin(init_cond.phi) - init_cond.t_dir[1],
        runtime_const.v0 * np.sin(init_cond.theta) - init_cond.t_dir[2],
    )


def air_density(z: float, atoms_model: AtmosphereModel) -> float:
    coef = np.power(1 - (z * atoms_model.lapse_rate / atoms_model.std_temp), atoms_model.index)
    return atoms_model.std_density * coef


def create_state_eq(runtime_const: RuntimeConst, atoms_model: AtmosphereModel) -> StateEquation:
    area = runtime_const.area
    cd = runtime_const.cd
    m = runtime_const.m
    match runtime_const.drag_type:
        case "pressure":
            drag_velocity_index = 1
        case "friction":
            drag_velocity_index = 0
        case _:
            raise ValueError(runtime_const.drag_type)

    def state_eq(_: float, state: State) -> tuple[float, float, float, float, float, float]:
        _, _, z, vx, vy, vz = state

        v_norm = np.sqrt(vx**2 + vy**2 + vz**2)
        k = area * cd * air_density(z, atoms_model) / 2  # 空気抵抗係数
        v_coef = k * np.power(v_norm, drag_velocity_index) / m

        ax = -v_coef * vx
        ay = -v_coef * vy
        az = -v_coef * vz - g / m
        return vx, vy, vz, ax, ay, az

    return state_eq


def solve(
    init_cond: InitCond,
    runtime_const: RuntimeConst,
    atoms_model: AtmosphereModel,
    event: TerminalEvent,
    t_eval: np.ndarray | None = None,
) -> TrajectoryResult:
    init_state = create_init_state(init_cond, runtime_const)
    sol = solve_ivp(
        create_state_eq(runtime_const, atoms_model),
        runtime_const.span,
        init_state,
        t_eval=t_eval,
        events=event,
        rtol=1e-8,
    )

    target_traj = reconstruct_target_trajectory(init_cond, sol.t)
    projectile_traj = sol.y[:3] + target_traj
    norm = np.linalg.norm(sol.y[:3], axis=0)
    min_index = np.argmin(norm)
    return TrajectoryResult(
        t=sol.t,
        target=target_traj,
        projectile=projectile_traj,
        approx_distance=norm[min_index],
        approx_index=int(min_index),
    )
