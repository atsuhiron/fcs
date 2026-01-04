import numpy as np

from fcs.algorithm.solver import solve
from fcs.algorithm.terminal_events import FallEvent
from fcs.plot_functions import plot_3d
from fcs.type import AtmosphereModel, InitCond, RuntimeConst

if __name__ == "__main__":
    rc = RuntimeConst(
        m=0.1,
        area=1e-3,
        cd=0.1,
        v0=300,
        span=(0, 10),
    )
    ic = InitCond(theta=np.deg2rad(60), phi=np.deg2rad(45), t_pos=(100, 100, 100), t_dir=(-10, 0, 0))
    am = AtmosphereModel()
    res = solve(ic, rc, am, FallEvent(ic))
    plot_3d(res)
