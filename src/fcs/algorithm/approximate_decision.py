import numpy as np

from fcs.algorithm.quartic_minimum import find_minimum_qurtic
from fcs.type import ApproximateParam, ApproximateResult, QuadraticParam, TrajectoryResult, Vector3

type N = float | np.ndarray


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


def rotate_xy[T: N](x: T, y: T, phi: float) -> tuple[T, T]:
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


def reconstruct_quadratic[T: N](q_param: QuadraticParam, t: T) -> T:
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


def find_approximation(
    approx_param: ApproximateParam,
    t_pos: Vector3,
    t_dir: Vector3,
) -> ApproximateResult:
    cos = np.cos(approx_param.phi)
    sin = np.sin(approx_param.phi)

    m11 = approx_param.x_param.a
    m12 = approx_param.x_param.b - t_dir[0] * cos - t_dir[1] * sin
    m13 = approx_param.x_param.c - t_pos[0] * cos - t_pos[1] * sin
    m22 = t_dir[0] * sin - t_dir[1] * cos
    m23 = t_pos[0] * sin - t_pos[1] * cos
    m31 = approx_param.z_param.a
    m32 = approx_param.z_param.b - t_dir[2]
    m33 = approx_param.z_param.c - t_pos[2]

    a4 = m11**2 + m31**2
    a3 = 2 * (m11 * m12 + m31 * m32)
    a2 = 2 * m11 * m13 + m12**2 + m22**2 + 2 * m31 * m33 + m32**2
    a1 = 2 * (m12 * m13 + m22 * m23 + m32 * m33)
    a0 = m13**2 + m23**2 + m33**2

    t_min, distance_min = find_minimum_qurtic(a4, a3, a2, a1, a0)
    target_pos = (
        t_pos[0] + t_min * t_dir[0],
        t_pos[1] + t_min * t_dir[1],
        t_pos[2] + t_min * t_dir[2],
    )

    proj_pos = rotate_xy(
        reconstruct_quadratic(approx_param.x_param, t_min),
        0,
        approx_param.phi,
    )
    return ApproximateResult(
        t=t_min,
        distance=distance_min,
        target_pos=target_pos,
        proj_pos=(proj_pos[0], proj_pos[1], reconstruct_quadratic(approx_param.z_param, t_min)),
    )
