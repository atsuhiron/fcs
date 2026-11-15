import numpy as np


def _q(x: np.ndarray, a4: float, a3: float, a2: float, a1: float, a0: float) -> np.ndarray:  # noqa: PLR0913
    return a4 * x**4 + a3 * x**3 + a2 * x**2 + a1 * x + a0


def find_minimum_qurtic(  # noqa: PLR0913
    a4: float,
    a3: float,
    a2: float,
    a1: float,
    a0: float,
    *,
    bounds: tuple[float, float] | None = None,
) -> tuple[float, float]:
    """
    Find the minimum of a quartic function.

    If `bounds` is given, the minimum is searched only within the closed interval [bounds[0], bounds[1]].
    """
    if a4 <= 0:
        raise ValueError

    # 導関数 (三次式) の根が極値の候補
    roots = np.roots([4 * a4, 3 * a3, 2 * a2, a1])

    # 実根かどうかを許容誤差で判定すると、重根が数値誤差で僅かに複素数になったときに取りこぼす。
    # 最小点は必ず実根に含まれ、それ以外の点の値が最小値を下回ることはないので、全ての根の実部を候補にしてよい
    candidates = roots.real
    if bounds is not None:
        # 区間内の最小点は、区間内の極値か区間の両端のいずれか
        lower, upper = bounds
        candidates = np.concatenate([np.clip(candidates, lower, upper), [lower, upper]])
    y = _q(candidates, a4, a3, a2, a1, a0)
    min_index = np.argmin(y)
    return float(candidates[min_index]), float(y[min_index])


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    a_arr = [0.1, -1, 1.5, -0.5, 0]
    a_arr[0] = np.abs(a_arr[0])

    xa = np.linspace(-2, 7, 100)
    min_x, min_y = find_minimum_qurtic(*a_arr)

    plt.plot(xa, _q(xa, *a_arr))
    plt.plot([min_x], _q(np.array([min_x]), *a_arr), "o")
    plt.show()
