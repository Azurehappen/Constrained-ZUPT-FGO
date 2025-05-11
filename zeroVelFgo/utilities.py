import numpy as np
from pyquaternion import Quaternion
from scipy.linalg import expm


class State:

    def __init__(
        self,
        pos_right: np.ndarray,
        vel_right: np.ndarray,
        quat_right: Quaternion,
        acc_bias_right: np.ndarray,
        gyro_bias_right: np.ndarray,
        pos_left: np.ndarray,
        vel_left: np.ndarray,
        quat_left: Quaternion,
        acc_bias_left: np.ndarray,
        gyro_bias_left: np.ndarray,
        error_cov: np.ndarray,
    ) -> None:
        self.pos_right = pos_right  # NED position: 3 by 1
        self.vel_right = vel_right  # NED position: 3 by 1
        self.quat_right = quat_right  # Quaternion from navigation to body frame: 4 by 1
        self.acc_bias_right = acc_bias_right  # Accelerometer bias: 3 by 1
        self.gyro_bias_right = gyro_bias_right  # Gyroscope bias: 3 by 1
        self.pos_left = pos_left  # NED position: 3 by 1
        self.vel_left = vel_left  # NED position: 3 by 1
        self.quat_left = quat_left  # Quaternion from navigation to body frame: 4 by 1
        self.acc_bias_left = acc_bias_left  # Accelerometer bias: 3 by 1
        self.gyro_bias_left = gyro_bias_left  # Gyroscope bias: 3 by 1
        self.error_cov = error_cov  # Error covariance matrix: 15 by 15


def getInitState(self) -> State:
    """
    Get the initial state for the smoothing process.
    """
    time_index = 0

    rot_right = ut.eulerToRotMat(-8.5, 0, -15)
    quat_right = Quaternion(matrix=rot_right)
    init_quat_right = np.array(quat_right.elements).reshape(4, 1)

    rot_left = ut.eulerToRotMat(-8.5, 0, -15)
    quat_left = Quaternion(matrix=rot_left)
    init_quat_left = np.array(quat_left.elements).reshape(4, 1)

    # Initialize the state
    pos_right = np.array([0.018, 0.016, -0.07]).reshape(3, 1)
    vel_right = np.zeros((3, 1))
    acc_bias_right = np.zeros((3, 1))
    gyro_bias_right = np.zeros((3, 1))

    pos_left = np.array([0.005, -0.305, -0.08]).reshape(3, 1)
    vel_left = np.zeros((3, 1))
    acc_bias_left = np.zeros((3, 1))
    gyro_bias_left = np.zeros((3, 1))

    error_cov = np.diag(
        [
            0.02**2,
            0.02**2,
            0.02**2,
            0.02**2,
            0.02**2,
            0.02**2,
            np.deg2rad(10) ** 2,
            np.deg2rad(10) ** 2,
            np.deg2rad(20) ** 2,
            2.5e-3,
            2.5e-3,
            2.5e-3,
            4e-6,
            4e-6,
            4e-6,
            0.02**2,
            0.02**2,
            0.02**2,
            0.02**2,
            0.02**2,
            0.02**2,
            np.deg2rad(10) ** 2,
            np.deg2rad(10) ** 2,
            np.deg2rad(20) ** 2,
            2.5e-3,
            2.5e-3,
            2.5e-3,
            4e-6,
            4e-6,
            4e-6,
        ]
    )

    return State(
        pos_right,
        vel_right,
        init_quat_right,
        acc_bias_right,
        gyro_bias_right,
        pos_left,
        vel_left,
        init_quat_left,
        acc_bias_left,
        gyro_bias_left,
        error_cov,
    )


def ecefToNedRotMat(lat_deg: float, lon_deg: float) -> np.ndarray:
    # Convert ECEF to NED
    lat_rad = np.deg2rad(lat_deg)
    lon_rad = np.deg2rad(lon_deg)

    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    C_e_n = np.array(
        [
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [-sin_lon, cos_lon, 0],
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
        ]
    )

    return C_e_n


def eulerToRotMat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """
    Convert Euler angles to rotation matrix. Follow the XYZ (roll_pitch_yaw) convention.
    Typically rotating from the navigation frame to the body frame.
    i.e., p_b = rot * p_n
    """
    roll_rad = np.deg2rad(roll_deg)
    pitch_rad = np.deg2rad(pitch_deg)
    yaw_rad = np.deg2rad(yaw_deg)

    cos_roll = np.cos(roll_rad)
    sin_roll = np.sin(roll_rad)
    cos_pitch = np.cos(pitch_rad)
    sin_pitch = np.sin(pitch_rad)
    cos_yaw = np.cos(yaw_rad)
    sin_yaw = np.sin(yaw_rad)

    R_x = np.array(
        [
            [1, 0, 0],
            [0, cos_roll, sin_roll],
            [0, -sin_roll, cos_roll],
        ]
    )
    R_y = np.array(
        [
            [cos_pitch, 0, -sin_pitch],
            [0, 1, 0],
            [sin_pitch, 0, cos_pitch],
        ]
    )
    R_z = np.array(
        [
            [cos_yaw, sin_yaw, 0],
            [-sin_yaw, cos_yaw, 0],
            [0, 0, 1],
        ]
    )

    return R_x @ R_y @ R_z


def rotationMatToQuat(rot_a2b: np.ndarray) -> np.ndarray:
    """
    Convert a rotation matrix to a normalized quaternion.
    For rotating frame a to frame b.

    Return: quat vecotr in 4 by 1.
    """

    b1 = 0.5 * np.sqrt(1 + rot_a2b[0, 0] + rot_a2b[1, 1] + rot_a2b[2, 2])
    b2 = (rot_a2b[2, 1] - rot_a2b[1, 2]) / (4 * b1)
    b3 = (rot_a2b[0, 2] - rot_a2b[2, 0]) / (4 * b1)
    b4 = (rot_a2b[1, 0] - rot_a2b[0, 1]) / (4 * b1)
    q_e2b = np.array([b1, b2, b3, b4])
    q_e2b = q_e2b / np.linalg.norm(q_e2b)
    return q_e2b.reshape(4, 1)


def vectorSkewSymMat(v: np.ndarray) -> np.ndarray:
    """
    Compute the skew symmetric matrix for the cross product.

    Parameters:
    v (np.ndarray): 3-by-1 vector

    Returns:
    np.ndarray: 3-by-3 skew symmetric matrix
    """
    v = v.flatten()
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def quatToRotationMat(q_a2b: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to rotation matrix

    Parameters:
    q_a2b (np.ndarray): 4-element quaternion vector = [a, bi, cj, dk]

    Returns:
    np.ndarray: 3-by-3 rotation matrix
    """
    q_a2b = q_a2b.flatten()
    if np.linalg.norm(q_a2b) != 0.0:
        q_a2b = q_a2b / np.linalg.norm(q_a2b)
        b1 = q_a2b[0]
        bv = q_a2b[1:4]
        rot_a2b = (
            (b1**2 - np.dot(bv, bv)) * np.eye(3)
            + 2 * np.outer(bv, bv)
            + 2 * b1 * vectorSkewSymMat(bv)
        )
        return rot_a2b
    else:
        raise ValueError("Norm b=0 in convertQuatToRot")


def quatToEulerAngleDegForN2b(q: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to Euler angles.
    Particular case when the quaternion is from navigation to body frame.

    Parameters:
    q (np.ndarray): 4-by-1 quaternion vector

    Returns:
    np.ndarray: 3-by-1 Euler angle vector
    """
    q = q.flatten()
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    roll = np.arctan2(2 * (q2 * q3 - q0 * q1), 1 - 2 * (q1**2 + q2**2))
    pitch = np.arcsin(-2 * (q1 * q3 + q0 * q2))
    yaw = np.arctan2(2 * (q1 * q2 - q0 * q3), 1 - 2 * (q2**2 + q3**2))

    return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)


def computeDiscreteModel(
    F: np.ndarray, G: np.ndarray, W: np.ndarray, dt: float
) -> tuple:
    """
    Compute the Phi and Qd increments.

    Parameters:
    F (np.ndarray): Continuous time state transition matrix
    G (np.ndarray): Continuous time process noise covariance matrix
    W (np.ndarray): Process noise covariance matrix
    dt (float): Time step duration

    Returns:
    tuple: Phi (np.ndarray), Qd (np.ndarray)
    """
    m, n = F.shape

    # Pre-compute the common matrix Q
    Q = G @ W @ G.T

    # Farrell eq. 4.113
    chi = np.block([[-F, Q], [np.zeros((m, m)), F.T]])

    # Farrell eq. 4.114
    gamma = expm(chi * dt)

    # Farrell eq. 4.115
    Phi = gamma[m : 2 * m, m : 2 * m].T

    # Farrell eq. 4.116
    Qd = Phi @ gamma[:m, m : 2 * m]

    return Phi, Qd


def quatInverse(q: np.ndarray) -> np.ndarray:
    """
    Compute the inverse of a quaternion.

    Parameters:
    q (np.ndarray): 4-by-1 quaternion vector

    Returns:
    np.ndarray: 4-by-1 quaternion vector
    """
    q = q.flatten()
    return (np.array([q[0], -q[1], -q[2], -q[3]]) / np.linalg.norm(q)).reshape(4, 1)


def quatMultiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Multiply two quaternions. Farrell eqn. D.3

    Parameters:
    a (np.ndarray): 4-by-1 quaternion vector
    b (np.ndarray): 4-by-1 quaternion vector

    Returns:
    np.ndarray: 4-by-1 quaternion vector
    """
    a = a.flatten()
    A = np.array(
        [
            [a[0], -a[1], -a[2], -a[3]],
            [a[1], a[0], -a[3], a[2]],
            [a[2], a[3], a[0], -a[1]],
            [a[3], -a[2], a[1], a[0]],
        ]
    )
    return A @ b
