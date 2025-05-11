import numpy as np
from dataclasses import dataclass
import gtsam
import yaml


def computeGravityConst(lat: float) -> float:
    g_h = (
        9.7803267715
        * (1 + 0.001931851353 * (np.sin(lat)) ** 2)
        / np.sqrt(1 - 0.0066943800229 * (np.sin(lat)) ** 2)
    )

    return g_h


def loadImuParamsFromYamlAxiswise(yaml_path: str):
    with open(yaml_path, "r") as f:
        params = yaml.safe_load(f)

    dt = 1.0 / params["update_rate"]

    acc_noise_std = np.array(
        [
            params["accelerometer_Xnoise_density"],
            params["accelerometer_Ynoise_density"],
            params["accelerometer_Znoise_density"],
        ]
    )  # * np.sqrt(dt)

    gyro_noise_std = np.array(
        [
            np.deg2rad(params["gyroscope_Xnoise_density"]),
            np.deg2rad(params["gyroscope_Ynoise_density"]),
            np.deg2rad(params["gyroscope_Znoise_density"]),
        ]
    )  # * np.sqrt(dt)

    acc_bias_std = np.array(
        [
            params["accelerometer_Xrandom_walk"],
            params["accelerometer_Yrandom_walk"],
            params["accelerometer_Zrandom_walk"],
        ]
    )  # * np.sqrt(dt)

    gyro_bias_std = np.array(
        [
            np.deg2rad(params["gyroscope_Xrandom_walk"]),
            np.deg2rad(params["gyroscope_Yrandom_walk"]),
            np.deg2rad(params["gyroscope_Zrandom_walk"]),
        ]
    )  # * np.sqrt(dt)

    return {
        "acc_noise_std": acc_noise_std,
        "gyro_noise_std": gyro_noise_std,
        "acc_bias_std": acc_bias_std,
        "gyro_bias_std": gyro_bias_std,
        "gravity": computeGravityConst(44.986656),
        "integration_sigma": np.array([1e-4, 1e-4, 1e-4]),
    }


@dataclass(frozen=True)
class ImuParams:
    acc_noise_std: np.ndarray  # shape (3,)
    gyro_noise_std: np.ndarray
    acc_bias_std: np.ndarray
    gyro_bias_std: np.ndarray
    gravity: float
    integration_sigma: float


@dataclass(frozen=True)
class InitialStateParams:
    POS_RIGHT = np.array([0.018, -0.016, 0.07])  # m
    POS_RIGHT_UNCERTAINTY = np.array([0.01, 0.01, 0.01])  # m

    POS_LEFT = np.array([0.005, 0.305, 0.08])
    POS_LEFT_UNCERTAINTY = np.array([0.01, 0.01, 0.01])

    VEL_RIGHT = np.array([0.0, 0.0, 0.0])  # m/s
    VEL_RIGHT_UNCERTAINTY = np.array([0.05, 0.05, 0.05])  # m/s

    VEL_LEFT = np.array([0.0, 0.0, 0.0])
    VEL_LEFT_UNCERTAINTY = np.array([0.05, 0.05, 0.05])

    # np.array([0.0, 0.0, 0.0]), np.array([-0.01088205, 0.00771465, -0.0005465])
    # ACC_BIAS_RIGHT = np.array([0.00, 0.00, 0.00])  # m/s^2
    # GYRO_BIAS_RIGHT = np.array([0.00, 0.00, 0.00])  # rad/s
    ACC_BIAS_RIGHT = np.array([0.05, -0.09, 0.02])  # m/s^2
    GYRO_BIAS_RIGHT = np.array([-0.009, 0.0077, -0.001])  # rad/s

    # np.array([0.0, 0.0, 0.0]), np.array([0.0063935, 0.0034922, -0.003318])
    # ACC_BIAS_LEFT = np.array([0.00, 0.00, 0.00])
    # GYRO_BIAS_LEFT = np.array([0.00, 0.00, 0.00])
    ACC_BIAS_LEFT = np.array([0.05, -0.02, 0.13])
    GYRO_BIAS_LEFT = np.array([0.005, 0.003, -0.003])

    # np.array([0.0424, 0.0424, 0.0424, 0.4275e-3, 0.4275e-3, 0.4275e-3])
    ACC_BIAS_UNCERTAINTY = np.array([0.01, 0.01, 0.01])  # m/s^2
    GYRO_BIAS_UNCERTAINTY = np.array([0.002, 0.002, 0.002])  # rad/s


class MeasurementParams:
    POS_NOISE_STD = 0.50  # m
    ZERO_NOISE_VEL_STD = 0.012  # m/s
    STEP_NOISE_LENGTH_STD = 0.15  # m

    MAX_FOOT_DISTANCE = 0.6  # m

    POS_MEAS_EPOCH_STEP = 60
    MAX_FOOT_EPOCH_STEP = 60
    ZUPT_EPOCH_STEP = 1


@dataclass(frozen=True)
class OneEpochResult:
    pos_right: gtsam.Point3
    pos_left: gtsam.Point3
    att_right: gtsam.Rot3
    att_left: gtsam.Rot3
    vel_right: np.ndarray  # Vector representation of velocity
    vel_left: np.ndarray
    bias_right: gtsam.imuBias.ConstantBias
    bias_left: gtsam.imuBias.ConstantBias
    pose_right_cov: np.ndarray  # Covariance is usually a matrix
    pose_left_cov: np.ndarray
    vel_right_cov: np.ndarray
    vel_left_cov: np.ndarray
    bias_right_cov: np.ndarray
    bias_left_cov: np.ndarray
