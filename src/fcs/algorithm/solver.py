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


def air_density(z: float, atmosphere_model: AtmosphereModel) -> float:
    coef = np.power(1 - (z * atmosphere_model.lapse_rate / atmosphere_model.std_temp), atmosphere_model.index_)
    return atmosphere_model.std_density * coef


def create_state_eq(
    runtime_const: RuntimeConst, atmosphere_model: AtmosphereModel, init_cond: InitCond
) -> StateEquation:
    area = runtime_const.area
    cd = runtime_const.cd
    m = runtime_const.m
    t_z0 = init_cond.t_pos[2]
    t_vx, t_vy, t_vz = init_cond.t_dir
    match runtime_const.drag_type:
        case "pressure":
            drag_velocity_index = 1
        case "friction":
            drag_velocity_index = 0
        case _:
            raise ValueError(runtime_const.drag_type)

    def state_eq(t: float, state: State) -> tuple[float, float, float, float, float, float]:
        _, _, z, vx, vy, vz = state

        # 状態は目標相対系なので、空気抵抗と大気密度は絶対系 (対気速度・絶対高度) に戻して計算する
        abs_vx = vx + t_vx
        abs_vy = vy + t_vy
        abs_vz = vz + t_vz
        abs_z = z + t_z0 + t_vz * t

        v_norm = np.sqrt(abs_vx**2 + abs_vy**2 + abs_vz**2)
        k = area * cd * air_density(abs_z, atmosphere_model) / 2  # 空気抵抗係数
        v_coef = k * np.power(v_norm, drag_velocity_index) / m

        # 目標は等速直線運動なので、相対加速度は絶対加速度に等しい
        ax = -v_coef * abs_vx
        ay = -v_coef * abs_vy
        az = -v_coef * abs_vz - g
        return vx, vy, vz, ax, ay, az

    return state_eq


def solve(
    init_cond: InitCond,
    runtime_const: RuntimeConst,
    atmosphere_model: AtmosphereModel,
    event: TerminalEvent,
    t_eval: np.ndarray | None = None,
) -> TrajectoryResult:
    init_state = create_init_state(init_cond, runtime_const)
    sol = solve_ivp(
        create_state_eq(runtime_const, atmosphere_model, init_cond),
        (0, runtime_const.span_end),
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
        target_traj=target_traj,
        projectile_traj=projectile_traj,
        approx_index=int(min_index),
    )
