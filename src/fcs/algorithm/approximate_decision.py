import numpy as np

from fcs.type import ApproximateParam, QuadraticParam, TrajectoryResult


def slice_adjacent(array: np.ndarray, target_index: int, width: int = 2) -> np.ndarray:
    """
    Slice an array around a target index with a specified width.

    Parameters
    ----------
    array : np.ndarray
        The input array to slice.
    target_index : int
        The index around which to slice the array.
    width : int, optional
        The number of elements to include on each side of the target index.
        Default is 2.

    Returns
    -------
    np.ndarray
        A slice of the array containing elements around the target index.

    Examples
    --------
    >>> import numpy as np
    >>> ar = np.arange(10)
    >>> _slice_adjacent(ar, 0)
    array([0, 1, 2])
    >>> _slice_adjacent(ar, 1)
    array([0, 1, 2, 3])
    >>> _slice_adjacent(ar, 2)
    array([0, 1, 2, 3, 4])
    >>> _slice_adjacent(ar, 3)
    array([1, 2, 3, 4, 5])
    """
    start_index = max(0, target_index - width)
    end_index = min(len(array), target_index + width + 1)
    return array[start_index:end_index]


def rotate_xy(x: np.ndarray, y: np.ndarray, phi: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Rotate coordinates (x, y) by angle phi.

    Parameters
    ----------
    x : np.ndarray
        x-coordinates.
    y : np.ndarray
        y-coordinates.
    phi : float
        Rotation angle in radians.

    Returns
    -------
    tuple[np.ndarray, np.ndarray]
        Rotated coordinates (x_rotated, y_rotated).
    """
    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)
    x_rotated = x * cos_phi - y * sin_phi
    y_rotated = x * sin_phi + y * cos_phi
    return x_rotated, y_rotated


def calc_quadratic_param(traj_result: TrajectoryResult) -> ApproximateParam:
    """
    Approximates the trajectory of the projectile
    near the closest point using a quadratic function and returns its parameters (a, b, c, phi).
    """
    x = slice_adjacent(traj_result.projectile_traj[0], traj_result.approx_index)
    y = slice_adjacent(traj_result.projectile_traj[1], traj_result.approx_index)
    z = slice_adjacent(traj_result.projectile_traj[2], traj_result.approx_index)
    t = slice_adjacent(traj_result.t, traj_result.approx_index)

    phi_arr = np.arctan2(y, x)
    phi = float(np.mean(phi_arr))
    phi_std = float(np.std(phi_arr))

    x_rotated, _ = rotate_xy(x, y, -phi)
    poly_param_x = np.polyfit(t, x_rotated, deg=2)
    poly_param_z = np.polyfit(t, z, deg=2)

    rmse_x = float(
        np.sqrt(np.mean((x_rotated - (poly_param_x[0] * t**2 + poly_param_x[1] * t + poly_param_x[2])) ** 2))
    )

    rmse_z = float(np.sqrt(np.mean((z - (poly_param_z[0] * t**2 + poly_param_z[1] * t + poly_param_z[2])) ** 2)))
    domain = (min(t), max(t))
    return ApproximateParam(
        x_param=QuadraticParam(
            a=float(poly_param_x[0]),
            b=float(poly_param_x[1]),
            c=float(poly_param_x[2]),
            domain=domain,
            rmse=rmse_x,
        ),
        z_param=QuadraticParam(
            a=float(poly_param_z[0]),
            b=float(poly_param_z[1]),
            c=float(poly_param_z[2]),
            domain=domain,
            rmse=rmse_z,
        ),
        phi=phi,
        phi_std=phi_std,
    )


def reconstruct_quadratic(q_param: QuadraticParam, t: np.ndarray) -> np.ndarray:
    """
    Reconstructs the quadratic function values at given time points.

    Parameters
    ----------
    q_param : QuadraticParam
        The parameters of the quadratic function.
    t : np.ndarray
        The time points at which to evaluate the quadratic function.

    Returns
    -------
    np.ndarray
        The values of the quadratic function at the specified time points.
    """
    return q_param.a * t**2 + q_param.b * t + q_param.c


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


def _find_minimum_qurtic(a4: float, a3: float, a2: float, a1: float, a0: float) -> tuple[float, float]:
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
    min_x, min_y = _find_minimum_qurtic(*a_arr)

    plt.plot(xa, _q(xa, *a_arr))
    plt.plot([min_x], _q(np.array([min_x]), *a_arr), "o")
    plt.show()
