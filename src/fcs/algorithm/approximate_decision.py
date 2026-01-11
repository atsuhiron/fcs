import numpy as np

from fcs.type import QuadraticParam, TrajectoryResult


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
    x_rotated = x * cos_phi + y * sin_phi
    y_rotated = -x * sin_phi + y * cos_phi
    return x_rotated, y_rotated


def calc_quadratic_param(traj_result: TrajectoryResult) -> QuadraticParam:
    """
    Approximates the trajectory of the projectile
    near the closest point using a quadratic function and returns its parameters (a, b, c, phi).
    """
    x = slice_adjacent(traj_result.projectile_traj[0], traj_result.approx_index)
    y = slice_adjacent(traj_result.projectile_traj[1], traj_result.approx_index)
    z = slice_adjacent(traj_result.projectile_traj[2], traj_result.approx_index)

    phi_arr = np.arctan2(y, x)
    phi = float(np.mean(phi_arr))
    phi_std = float(np.std(phi_arr))

    x_rotated, _ = rotate_xy(x, y, -phi)
    poly_param = np.polyfit(x_rotated, z, deg=2)

    rmse = float(
        np.sqrt(
            np.mean((z - (poly_param[0] * x_rotated**2 + poly_param[1] * x_rotated + poly_param[2]))**2)
        )
    )

    return QuadraticParam(
        a=poly_param[0],
        b=poly_param[1],
        c=poly_param[2],
        phi=phi,
        phi_std=phi_std,
        rx_dmain=(min(x_rotated), max(x_rotated)),
        rmse=rmse,
    )
