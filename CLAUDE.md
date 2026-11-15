# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`fcs` is a fire-control (lead) calculator: given projectile specs and a target's position and constant velocity, it finds the azimuth/elevation that makes the projectile's trajectory (with altitude-dependent air drag) pass closest to the target. `README.md` holds the full mathematical derivation and is the specification for the algorithm — keep it in sync when the maths changes.

## Commands

Python 3.13, managed with `uv`. The package uses a `src/` layout and must be installed (which `uv sync` / `uv run` do) for the `fcs.*` imports to resolve.

```shell
uv sync --group dev        # dev tools (ruff, ty) are NOT installed by a plain `uv sync` (default-groups = [])
check.bat                  # ruff format + ruff check --fix + ty check (modifies files)
uv run ruff format --check # non-modifying equivalents
uv run ruff check
uv run ty check
```

Running:

```shell
# positional args: mass area drag_coefficient initial_velocity target_x target_y target_z direction_x direction_y direction_z
uv run lead 0.1 0.001 0.1 300 100 100 100 -10 1 0        # -> "phi_deg theta_deg distance eta_msec calc_msec"
uv run lead -r --plot 0.1 0.001 0.1 300 100 100 100 -10 1 0   # human-readable output + 3D matplotlib window
uv run python -m fcs.scripts.show_trajectory_samples     # drag-model comparison plot (source of images/trajectory_sample.png)
uv run python -m fcs.algorithm.quartic_minimum           # visual sanity check of the quartic minimiser
```

There is no test suite. The README example above (expected `46.95 36.46 0.00 601.31 <calc_time>`) is the only regression reference. `--plot` and the scripts open blocking matplotlib windows.

## Architecture

The computation is three nested layers, mirroring README "Procedure" steps 2–4. All data passed between them are `NamedTuple`s defined in `src/fcs/type.py`.

1. **`algorithm/solver.py` — trajectory ODE.** `solve()` integrates the 6-state equation with `scipy.integrate.solve_ivp`. The state is in the **target-relative frame**: initial position is `-t_pos` and the target velocity is subtracted from the muzzle velocity, so the norm of the state position is directly the projectile–target distance. Drag is the exception: the state equation converts back to the projectile's absolute velocity and absolute altitude before computing it, because drag depends on motion through the air, not relative to the target. `solve()` then adds the reconstructed target path back (`util.reconstruct_target_trajectory`) so `TrajectoryResult.projectile_traj` / `target_traj` are in absolute coordinates, and records `approx_index`, the sample with the smallest distance. Integration stops via a `TerminalEvent` (`algorithm/terminal_events.py`); `FallEvent` fires when the projectile's absolute z crosses zero downward; it returns a positive constant at t = 0 so that the launch itself (also at z = 0) is not taken for a landing.
2. **`algorithm/approximate_decision.py` — sub-sample closest approach.** The ODE output is discrete, so `calc_quadratic_param()` takes the ±2 samples around `approx_index`, rotates them about z by the mean azimuth into a plane, and fits x'(t) and z(t) with quadratics. `find_approximation()` expands |projectile − target|² into a quartic in t and minimises it with `algorithm/quartic_minimum.py` (roots of the derivative via `np.roots`), restricted to the fitted time window: outside it the quadratic would extrapolate to a trajectory that does not exist (after landing or after `span_end`). The quartic is the squared distance; `ApproximateResult.distance` is its square root, clamped at 0 because the fit can dip slightly negative. Callers use `find_closest_point()`, which wraps both steps and returns the closest sample itself when the trajectory has fewer than 3 samples (elevation near 0, where the projectile lands at once).
3. **`algorithm/optimizer.py` — aim search.** `run_optimization()` runs `scipy.optimize.minimize` (L-BFGS-B) over `(phi, theta)`, where each objective evaluation runs layers 1 and 2 in full. The objective is the squared distance (smooth at a hit). Azimuth is unbounded and wrapped into `(-π, π]` afterwards; elevation is searched over `[0, π]` and folded back into `[0, π/2]` (with azimuth + π) afterwards. The lower bound is 0 because the shooter sits on the ground at the origin; the upper bound is π rather than π/2 because the zenith is a singularity where azimuth has no effect, and a search that reaches it with the azimuth pointing the wrong way cannot leave unless it may pass through. The initial guess points straight at the target's current position. The search is capped at 300 function evaluations (hits converge in under 200; targets out of reach otherwise take seconds). `OptimizeResult.status` is decided from the closest distance and `tolerance`, not from SciPy's `result.success`, which does not correspond to a hit: `hit`, `out_of_time` (trajectory cut off at `span_end` before landing) or `out_of_range` (landed short).

`main.py` (`calc_lead`, exposed as the `lead` console script) parses arguments, rejects invalid values before any calculation (`_find_invalid_inputs`, exit code 2 via `parser.error`), calls `run_optimization`, exits with a status-specific code (0 hit, 3 `out_of_range`, 4 `out_of_time`; the stdout format is the same in all three and the reason goes to stderr), and for `--plot` re-solves once at the optimum to feed `plot_functions.plot_3d`.

### Things that are easy to get wrong

- Angle order differs between APIs: the optimizer vector and `OptimizeResult` are `(phi, theta)`; `InitCond` is `(theta, phi, ...)`. Use keyword arguments.
- `drag_type="pressure"` means drag ∝ |v|² (velocity exponent 1 in the coefficient); `"friction"` means drag ∝ |v| (exponent 0).
- Launch is always at t = 0: `RuntimeConst.span_end` is only the maximum flight time to simulate, and the initial state assumes the target is at `t_pos` at launch.
- `find_minimum_qurtic` is the (misspelled) public name; it raises `ValueError` if the leading coefficient is not positive.

## Conventions

- Ruff runs with `select = ["ALL"]` (only `D` and `COM812` ignored), line length 119. New code needs full type annotations, and deliberate violations get a targeted `# noqa: <code>` (e.g. `T201` for the CLI's `print`).
- Code uses Python 3.12+ syntax: `type` aliases and PEP 695 generics (`def f[T: N](...)`).
- Some inline comments are in Japanese; leave them as they are.
