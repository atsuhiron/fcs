import argparse

from fcs.algorithm.approximate_decision import calc_quadratic_param, find_approximation
from fcs.algorithm.optimizer import run_optimization
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.plot_functions import plot_3d
from fcs.type import AtmosphereModel, InitCond, RuntimeConst


def calc_lead() -> None:
    parser = argparse.ArgumentParser(description="Fire Control System Simulation")
    parser.add_argument("mass", type=float, help="Mass of the projectile (kg)")
    parser.add_argument("area", type=float, help="Cross-sectional area of projectile (m^2)")
    parser.add_argument("drag_coefficient", type=float, help="Drag coefficient of projectile")
    parser.add_argument("initial_velocity", type=float, help="Initial velocity of projectile (m/s)")
    parser.add_argument("target_x", type=float, help="Target X position (m)")
    parser.add_argument("target_y", type=float, help="Target Y position (m)")
    parser.add_argument("target_z", type=float, help="Target Z position (m)")
    parser.add_argument("direction_x", type=float, help="Target direction X component")
    parser.add_argument("direction_y", type=float, help="Target direction Y component")
    parser.add_argument("direction_z", type=float, help="Target direction Z component")
    parser.add_argument("--span-start", type=float, default=0, help="Start of the time span (s)")
    parser.add_argument("--span-end", type=float, default=10, help="End of the time span (s)")
    parser.add_argument("--std-density", type=float, default=1.225, help="Standard atmospheric density (kg/m^3)")
    parser.add_argument("--std-temp", type=float, default=288.15, help="Standard atmospheric temperature (K)")
    parser.add_argument("--lapse-rate", type=float, default=0.0065, help="Atmospheric lapse rate (K/m)")
    parser.add_argument("--lapse-index", type=float, default=5.256, help="Atmospheric index")
    parser.add_argument(
        "--drag-type",
        type=str,
        choices=["pressure", "friction"],
        default="pressure",
        help="Type of drag model to use",
    )
    parser.add_argument("--plot", action="store_true", help="Enable 3D plotting of the trajectory")
    parser.add_argument("--readable", "-r", action="store_true", help="Output in a human-readable format")

    args = parser.parse_args()

    runtime_const = RuntimeConst(
        m=args.mass,
        area=args.area,
        cd=args.drag_coefficient,
        v0=args.initial_velocity,
        span=(args.span_start, args.span_end),
        drag_type=args.drag_type,
    )
    t_pos = (args.target_x, args.target_y, args.target_z)
    t_dir = (args.direction_x, args.direction_y, args.direction_z)

    specifications = run_optimization(runtime_const, t_pos, t_dir)
    if args.readable:
        print(specifications.format_h())  # noqa: T201
    else:
        print(specifications.format_cl())  # noqa: T201

    if args.plot:
        ic = InitCond(theta=specifications.theta, phi=specifications.phi, t_pos=t_pos, t_dir=t_dir)
        am = AtmosphereModel(
            std_density=args.std_density,
            std_temp=args.std_temp,
            lapse_rate=args.lapse_rate,
            index_=args.lapse_index,
        )
        res = solve(ic, runtime_const, am, FallEvent(ic))
        aprox_param = calc_quadratic_param(res)
        approx_result = find_approximation(aprox_param, ic.t_pos, ic.t_dir)
        plot_3d(res, approx_result)


if __name__ == "__main__":
    calc_lead()
