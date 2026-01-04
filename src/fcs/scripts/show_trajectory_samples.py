import numpy as np

from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.plot_functions import plot_xz
from fcs.type import AtmosphereModel, InitCond, RuntimeConst

ic = InitCond(theta=np.deg2rad(60), phi=0, t_pos=(100, 100, 100), t_dir=(0, 0, 0))
terminal_ev = FallEvent(ic)
span = (0, 10)
v0 = 400
params = [
    (
        RuntimeConst(m=0.1, area=1e-3, cd=0, v0=v0, span=span),
        AtmosphereModel(lapse_rate=0),
        "no_drag (cd=0.0)",
    ),
    (
        RuntimeConst(m=0.1, area=1e-3, cd=3, v0=v0, span=span, drag_type="friction"),
        AtmosphereModel(lapse_rate=0),
        "friction_drag_isotropic_atmosphere (cd=3.0)",
    ),
    (
        RuntimeConst(m=0.1, area=1e-3, cd=0.1, v0=v0, span=span),
        AtmosphereModel(lapse_rate=0),
        "pressure_drag_isotropic_atmosphere (cd=0.1)",
    ),
    (
        RuntimeConst(m=0.1, area=1e-3, cd=0.1, v0=v0, span=span),
        AtmosphereModel(),
        "pressure_drag_lapse_atmosphere (cd=0.1)",
    ),
]

if __name__ == "__main__":
    results = []
    names = []
    for param in params:
        sol = solve(ic, param[0], param[1], terminal_ev, np.linspace(span[0], span[1], 1024))
        results.append(sol)
        names.append(param[2])
    plot_xz(results, names)
