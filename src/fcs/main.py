import argparse
import math
import sys

from fcs.algorithm.approximate_decision import find_closest_point
from fcs.algorithm.optimizer import run_optimization
from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.plot_functions import plot_3d
from fcs.type import AtmosphereModel, InitCond, OptimizeStatus, RuntimeConst

# 0: 命中, 1: 予期しない例外 (Python の既定), 2: 引数の誤り (argparse の既定)
EXIT_CODES: dict[OptimizeStatus, int] = {
    "hit": 0,
    "out_of_range": 3,
    "out_of_time": 4,
}
STATUS_MESSAGES: dict[OptimizeStatus, str] = {
    "hit": "the projectile hits the target",
    "out_of_range": "the projectile cannot reach the target",
    "out_of_time": "the projectile does not reach the target within --span-end",
}


def _find_invalid_inputs(args: argparse.Namespace) -> list[str]:
    """Return a message for each argument whose value makes the calculation meaningless."""
    errors = [
        f"{name} must be a finite number: {value}" for name, value in vars(args).items() if _is_non_finite(value)
    ]
    if errors:
        return errors

    # 各行は、引数名・値・満たすべき条件・条件の説明の順
    rules: list[tuple[str, float, bool, str]] = [
        ("mass", args.mass, args.mass > 0, "greater than 0"),
        ("area", args.area, args.area > 0, "greater than 0"),
        ("drag_coefficient", args.drag_coefficient, args.drag_coefficient >= 0, "0 or greater"),
        ("initial_velocity", args.initial_velocity, args.initial_velocity > 0, "greater than 0"),
        # 射手は地面 (z = 0) にいて、弾は z < 0 に入れない
        ("target_z", args.target_z, args.target_z >= 0, "0 or greater"),
        ("--span-end", args.span_end, args.span_end > 0, "greater than 0"),
        ("--tolerance", args.tolerance, args.tolerance >= 0, "0 or greater"),
        ("--std-density", args.std_density, args.std_density >= 0, "0 or greater"),
        ("--std-temp", args.std_temp, args.std_temp > 0, "greater than 0"),
        ("--lapse-rate", args.lapse_rate, args.lapse_rate >= 0, "0 or greater"),
    ]
    errors = [f"{name} must be {condition}: {value}" for name, value, ok, condition in rules if not ok]

    if args.target_x == 0 and args.target_y == 0 and args.target_z == 0:
        errors.append("the target must not be at the launch point (0, 0, 0)")
    return errors


def _is_non_finite(value: object) -> bool:
    return isinstance(value, float) and not math.isfinite(value)


def calc_lead() -> None:
    parser = argparse.ArgumentParser(
        description="Fire Control System Simulation",
        epilog=(
            "exit status: 0 hit, 1 unexpected error, 2 invalid arguments, "
            "3 out_of_range (the projectile cannot reach the target), "
            "4 out_of_time (it does not reach the target within --span-end). "
            'For 3 and 4 the result is still printed and "lead: <status>: <message>" is written to stderr.'
        ),
    )
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
    parser.add_argument("--span-end", type=float, default=10, help="Maximum flight time to simulate (s)")
    parser.add_argument(
        "--tolerance",
        type=float,
        default=0.1,
        help="Closest distance to the target that counts as a hit (m)",
    )
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
    if errors := _find_invalid_inputs(args):
        parser.error("; ".join(errors))  # 終了コード 2 で終了する

    runtime_const = RuntimeConst(
        m=args.mass,
        area=args.area,
        cd=args.drag_coefficient,
        v0=args.initial_velocity,
        span_end=args.span_end,
        drag_type=args.drag_type,
    )
    am = AtmosphereModel(
        std_density=args.std_density,
        std_temp=args.std_temp,
        lapse_rate=args.lapse_rate,
        index_=args.lapse_index,
    )
    t_pos = (args.target_x, args.target_y, args.target_z)
    t_dir = (args.direction_x, args.direction_y, args.direction_z)

    specifications = run_optimization(runtime_const, t_pos, t_dir, am, args.tolerance)
    if args.readable:
        print(specifications.format_h())  # noqa: T201
    else:
        print(specifications.format_cl())  # noqa: T201

    if args.plot:
        ic = InitCond(theta=specifications.theta, phi=specifications.phi, t_pos=t_pos, t_dir=t_dir)
        res = solve(ic, runtime_const, am, FallEvent(ic))
        approx_result = find_closest_point(res, ic.t_pos, ic.t_dir)
        plot_3d(res, approx_result)

    if specifications.status != "hit":
        sys.stderr.write(f"lead: {specifications.status}: {STATUS_MESSAGES[specifications.status]}\n")
        sys.exit(EXIT_CODES[specifications.status])


if __name__ == "__main__":
    calc_lead()
