from fcs.algorithm.approximate_decision import calc_quadratic_param, find_approximation
from fcs.algorithm.optimizer import run
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.plot_functions import plot_3d
from fcs.type import AtmosphereModel, InitCond, RuntimeConst

if __name__ == "__main__":
    runtime_const = RuntimeConst(
        m=0.1,
        area=1e-3,
        cd=0.1,
        v0=300,
        span=(0, 10),
    )
    t_pos = (100, 100, 100)
    t_dir = (-10, 1, 0)

    specifications = run(runtime_const, t_pos, t_dir)
    print(specifications)  # noqa: T201

    ic = InitCond(theta=specifications.theta, phi=specifications.phi, t_pos=t_pos, t_dir=t_dir)
    am = AtmosphereModel()
    res = solve(ic, runtime_const, am, FallEvent(ic))
    aprox_param = calc_quadratic_param(res)
    approx_result = find_approximation(aprox_param, ic.t_pos, ic.t_dir)
    plot_3d(res, approx_result)
