import numpy as np

from fcs.type import InitCond


def reconstruct_target_trajectory(init_cond: InitCond, t_arr: np.ndarray) -> np.ndarray:
    move = np.array(init_cond.t_dir)[np.newaxis, :] * t_arr[:, np.newaxis]
    init = np.array(init_cond.t_pos)[np.newaxis, :]
    return np.transpose(move + init)
