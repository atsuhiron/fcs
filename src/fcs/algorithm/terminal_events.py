import abc

import numpy as np

from fcs.type import InitCond, State
from fcs.util import reconstruct_target_trajectory


class TerminalEvent(abc.ABC):
    @abc.abstractmethod
    def __call__(self, t: float, state: State, *__: object) -> float:
        pass

    @property
    @abc.abstractmethod
    def terminal(self) -> bool:
        pass

    @property
    @abc.abstractmethod
    def direction(self) -> int:
        pass


class FallEvent(TerminalEvent):
    def __init__(self, init_cond: InitCond) -> None:
        self._init_cond = init_cond

    def __call__(self, t: float, state: State, *__: object) -> float:
        if t <= 0:
            # 発射点は z = 0 なので、発射の瞬間を着地とみなさないよう正の値を返す
            return 1.0
        target = reconstruct_target_trajectory(self._init_cond, np.array([t]))
        return state[2] + target[2, 0]  # z が負になったら終了

    @property
    def terminal(self) -> bool:
        return True

    @property
    def direction(self) -> int:
        return -1  # pos -> neg
