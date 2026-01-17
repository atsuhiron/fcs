import numpy as np


def _solve_cubic_eq(b3: complex, b2: complex, b1: complex, b0: complex) -> tuple[complex, complex, complex]:
    c1 = -2 * b2**3 + 9 * b1 * b2 * b3 - 27 * b0 * b3**2
    c4 = -(b2**2) + 3 * b1 * b3
    c3 = c1**2 + 4 * c4**3
    c2 = np.power(c1 + np.sqrt(c3), 1 / 3)

    c5 = -b2 / 3 / b3
    c6 = 3 * np.power(2, 1 / 3) * b3
    c7 = complex(0, float(np.sqrt(3)))

    x1 = c5 + c2 / c6 - np.power(2, 1 / 3) * c4 / (3 * c2 * b3)
    x2 = c5 - (1 - c7) * c2 / (2 * c6) + (1 + c7) * c4 / (3 * np.power(2, 2 / 3) * c2 * b3)
    x3 = c5 - (1 + c7) * c2 / (2 * c6) + (1 - c7) * c4 / (3 * np.power(2, 2 / 3) * c2 * b3)

    return x1, x2, x3


def _q(x: np.ndarray, a4: float, a3: float, a2: float, a1: float, a0: float) -> np.ndarray:  # noqa: PLR0913
    return a4 * x**4 + a3 * x**3 + a2 * x**2 + a1 * x + a0


def find_minimum_qurtic(a4: float, a3: float, a2: float, a1: float, a0: float) -> tuple[float, float]:
    if a4 <= 0:
        raise ValueError

    # differential coef
    b3 = complex(4 * a4)
    b2 = complex(3 * a3)
    b1 = complex(2 * a2)
    b0 = complex(a1)

    real_roots = np.array([x.real for x in _solve_cubic_eq(b3, b2, b1, b0) if np.isclose(x.imag, 0)])
    y = _q(real_roots, a4, a3, a2, a1, a0)
    min_index = np.argmin(y)
    return real_roots[min_index], y[min_index]


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    a_arr = [0.1, -1, 1.5, -0.5, 0]
    a_arr[0] = np.abs(a_arr[0])

    xa = np.linspace(-2, 7, 100)
    min_x, min_y = find_minimum_qurtic(*a_arr)

    plt.plot(xa, _q(xa, *a_arr))
    plt.plot([min_x], _q(np.array([min_x]), *a_arr), "o")
    plt.show()
