from enum import Enum
from functools import partial
import numpy as np
import pandas as pd
from typing import List, Optional
from tqdm import tqdm
from collections import deque
from collections import OrderedDict
from parameters import ImuParams, InitialStateParams, MeasurementParams, OneEpochResult
from parameters import computeGravityConst, loadImuParamsFromYamlAxiswise
import os

# import boule
import gtsam
import gtsam_unstable

from gtsam import (
    NonlinearFactorGraph,
    Values,
    Pose3,
    PriorFactorPose3,
    PriorFactorVector,
    BetweenFactorPose3,
    ImuFactor,
    PreintegratedImuMeasurements,
    PreintegrationParams,
    noiseModel,
)


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


class MeasType(Enum):
    Position = 1
    ZeroVelocity = 2
    Both = 3
    Initial = 4
    NoMeasurement = 5


class GtsamOptimizerType:
    LevenbergMarquard = 1
    ISAM2 = 2
    ISAM2_FIXED_LAG = 3


class ZeroVelFgoEstimator:

    def __init__(
        self,
        imu_right_file: str,
        imu_left_file: str,
        position_meas_file: str,
        use_pos_meas: bool = True,
        use_max_foot: bool = True,
        use_zero_vel: bool = True,
        gtsam_optimizer_type: GtsamOptimizerType = GtsamOptimizerType.LevenbergMarquard,
    ):
        # Load IMU and Position Data
        self.imu_right_pd = pd.read_csv(imu_right_file)
        self.imu_left_pd = pd.read_csv(imu_left_file)
        self.position_meas_pd = pd.read_csv(position_meas_file)

        self.gtsam_optimizer_type = gtsam_optimizer_type

        self.use_pos_meas = use_pos_meas
        self.use_max_foot = use_max_foot
        self.use_zero_vel = use_zero_vel

        base_dir = os.path.dirname(__file__)
        self.dt = 1.0 / 60.0

        right_yaml = os.path.join(base_dir, "../measData/customized_all_right_imu.yaml")
        left_yaml = os.path.join(base_dir, "../measData/customized_all_left_imu.yaml")

        self.imu_params_right = ImuParams(**loadImuParamsFromYamlAxiswise(right_yaml))
        self.imu_params_left = ImuParams(**loadImuParamsFromYamlAxiswise(left_yaml))

        self.gtsam_params_right = gtsam.PreintegrationParams.MakeSharedU(
            self.imu_params_right.gravity
        )
        self.gtsam_params_right.setGyroscopeCovariance(
            np.diag((self.imu_params_right.gyro_noise_std) ** 2)
        )
        self.gtsam_params_right.setAccelerometerCovariance(
            np.diag((self.imu_params_right.acc_noise_std) ** 2)
        )
        self.gtsam_params_right.setIntegrationCovariance(
            np.diag(self.imu_params_right.integration_sigma**2)
        )

        self.gtsam_params_left = gtsam.PreintegrationParams.MakeSharedU(
            self.imu_params_left.gravity
        )
        self.gtsam_params_left.setGyroscopeCovariance(
            np.diag((self.imu_params_left.gyro_noise_std) ** 2)
        )
        self.gtsam_params_left.setAccelerometerCovariance(
            np.diag((self.imu_params_left.acc_noise_std) ** 2)
        )
        self.gtsam_params_left.setIntegrationCovariance(
            np.diag(self.imu_params_left.integration_sigma**2)
        )

        # Noise model for step length constraint
        self.step_length_noise = noiseModel.Isotropic.Sigma(
            1, MeasurementParams.STEP_NOISE_LENGTH_STD
        )

        self.zero_vel_noise = noiseModel.Isotropic.Sigma(
            3, MeasurementParams.ZERO_NOISE_VEL_STD
        )  # Velocity measurement

        self.pose_meas_noise = gtsam.noiseModel.Diagonal.Precisions(
            np.asarray([0, 0, 0] + [1.0 / (MeasurementParams.POS_NOISE_STD**2)] * 3)
        )  # Orientation information + position information

        # Initial states
        # Initial states (Separate for Left and Right Foot)
        self.current_pose_right = Pose3(
            gtsam.Rot3(),
            InitialStateParams.POS_RIGHT,
        )  # Identity pose (constains orientation and position)
        self.current_pose_left = Pose3(
            gtsam.Rot3(),
            InitialStateParams.POS_LEFT,
        )

        self.current_velocity_right = InitialStateParams.VEL_RIGHT  # Initial velocity
        self.current_velocity_left = InitialStateParams.VEL_LEFT  # Initial velocity

        self.current_bias_right = gtsam.imuBias.ConstantBias(
            InitialStateParams.ACC_BIAS_RIGHT,
            InitialStateParams.GYRO_BIAS_RIGHT,
        )
        self.current_bias_left = gtsam.imuBias.ConstantBias(
            InitialStateParams.ACC_BIAS_LEFT,
            InitialStateParams.GYRO_BIAS_LEFT,
        )

        self.init_bias_noise = noiseModel.Diagonal.Sigmas(
            np.concatenate(
                (
                    InitialStateParams.ACC_BIAS_UNCERTAINTY,
                    InitialStateParams.GYRO_BIAS_UNCERTAINTY,
                )
            )
        )

        # Noise models
        self.init_pose_noise = noiseModel.Diagonal.Precisions(
            np.asarray(
                [
                    1 / (np.deg2rad(10) ** 2),
                    1 / (np.deg2rad(10) ** 2),
                    1 / (np.deg2rad(10) ** 2),
                ]
                + [1.0 / (0.01**2)] * 3
            )
        )  # Pose
        self.init_vel_noise = noiseModel.Isotropic.Sigma(3, 0.05)  # Velocity

        # Add prior factors
        self.curr_pose_right_key = 0
        self.curr_pose_left_key = 1
        self.curr_vel_right_key = 2
        self.curr_vel_left_key = 3
        self.curr_bias_right_key = 4
        self.curr_bias_left_key = 5

        self.epoch_key_dict = OrderedDict()
        self.epoch_factor_index_dict = OrderedDict()

        self.new_factors = gtsam.NonlinearFactorGraph()
        self.new_factors.addPriorPose3(
            self.curr_pose_right_key, self.current_pose_right, self.init_pose_noise
        )
        self.new_factors.addPriorPose3(
            self.curr_pose_left_key, self.current_pose_left, self.init_pose_noise
        )
        self.new_factors.addPriorVector(
            self.curr_vel_right_key, self.current_velocity_right, self.init_vel_noise
        )
        self.new_factors.addPriorVector(
            self.curr_vel_left_key, self.current_velocity_left, self.init_vel_noise
        )
        self.new_factors.addPriorConstantBias(
            self.curr_bias_right_key, self.current_bias_right, self.init_bias_noise
        )
        self.new_factors.addPriorConstantBias(
            self.curr_bias_left_key, self.current_bias_left, self.init_bias_noise
        )

        self.epoch_i = 0
        self.prev_epoch_i = None

        self.curr_factor_index = 5
        self.epoch_factor_index_dict[self.epoch_i] = list(
            range(6)
        )  # Store (time, keys)

        # Insert initial values (initial estimates for optimization)
        self.new_values = gtsam.Values()
        self.new_values.insert(self.curr_pose_right_key, self.current_pose_right)
        self.new_values.insert(self.curr_pose_left_key, self.current_pose_left)
        self.new_values.insert(self.curr_vel_right_key, self.current_velocity_right)
        self.new_values.insert(self.curr_vel_left_key, self.current_velocity_left)
        self.new_values.insert(self.curr_bias_right_key, self.current_bias_right)
        self.new_values.insert(self.curr_bias_left_key, self.current_bias_left)

        self.epoch_key_dict[self.epoch_i] = list(range(6))  # Store (time, keys)

        if self.gtsam_optimizer_type == GtsamOptimizerType.ISAM2_FIXED_LAG:
            self.optimizer = gtsam_unstable.IncrementalFixedLagSmoother(5)
        elif self.gtsam_optimizer_type == GtsamOptimizerType.ISAM2:
            isam_params = gtsam.ISAM2Params()
            isam_params.setFactorization("CHOLESKY")
            isam_params.relinearizeSkip = 15
            self.optimizer = gtsam.ISAM2(isam_params)
        elif self.gtsam_optimizer_type == GtsamOptimizerType.LevenbergMarquard:
            self.optimizer = gtsam.LevenbergMarquardtOptimizer(
                self.new_factors, self.new_values
            )
            self.marginals = None
        else:
            raise ValueError("Invalid GTSAM optimizer selected.")

        self.gtsam_results = gtsam.Values()

        self.total_num_est_epochs = 1

        self.timeIndexToFirstPoseKey = {}
        self.timeIndexToFirstPoseKey[0] = (
            self.curr_pose_right_key,
            MeasType.Initial,
        )

        self.smoothing_results = {}
        self.realtime_results = {}

        self.max_foot_distance = MeasurementParams.MAX_FOOT_DISTANCE

    # Define the error function for the GPS factor
    @staticmethod
    def error_pos(
        measurement: np.ndarray,
        this: gtsam.CustomFactor,
        values: gtsam.Values,
        jacobians: Optional[List[np.ndarray]],
    ) -> np.ndarray:
        """Position Factor error function
        :param measurement: pos measurement, to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        key = this.keys()[0]
        pos1 = values.atPose3(key).translation()
        error = pos1 - measurement
        if jacobians is not None:
            jacobians[0] = np.eye(3)

        return error

    def process_pos_measurements(
        self,
        k: int,
        is_valid_pos_meas: bool,
        pos_match: pd.DataFrame,
        curr_pose_right_key: int,
        curr_pose_left_key: int,
    ) -> None:
        """
        Process position and nonlinear step-length measurements.

        Args:
            k (int): Current epoch index.
            new_factors (NonlinearFactorGraph): Factor graph storing new constraints.
            new_values (Values): Current estimated values of states.
            pos_match (np.ndarray): Position measurement data.
            curr_pose_right_key (int): Key for the right foot pose.
            curr_pose_left_key (int): Key for the left foot pose.

        Returns:
            NonlinearFactorGraph: Updated factor graph with new constraints.
        """
        # Add position measurement factors
        if not is_valid_pos_meas:
            return

        row = pos_match.iloc[0]  # Only one match is expected
        # Need to convert global frame orientation to body frame orientation
        # TODO: verify if GTSAM uses z-axis up to represent the orientation.
        #       The measurent is represented in z-axis down frame.
        pos_left_meas = gtsam.Point3(row.pos_left_x, -row.pos_left_y, -row.pos_left_z)
        pos_right_meas = gtsam.Point3(
            row.pos_right_x, -row.pos_right_y, -row.pos_right_z
        )

        self.new_factors.push_back(
            gtsam.PriorFactorPose3(
                curr_pose_right_key,
                gtsam.Pose3(self.current_pose_right.rotation(), pos_right_meas),
                self.pose_meas_noise,
            )
        )
        self.insert_epoch_factor_dict(self.epoch_i)

        self.new_factors.push_back(
            gtsam.PriorFactorPose3(
                curr_pose_left_key,
                gtsam.Pose3(self.current_pose_left.rotation(), pos_left_meas),
                self.pose_meas_noise,
            )
        )
        self.insert_epoch_factor_dict(self.epoch_i)

    # Define the nonlinear error function enforcing the step length constraint
    @staticmethod
    def error_step_length(
        measurement: float,
        this: gtsam.CustomFactor,
        values: gtsam.Values,
        jacobians: Optional[List[np.ndarray]],
    ) -> np.ndarray:
        """
        Nonlinear error function enforcing the step length constraint.
        Computes the residual enforcing the step length condition between the left and right foot poses.

        Args:
            measurement (float): Expected step length (self.max_foot_distance meters).
            this (CustomFactor): GTSAM custom factor instance.
            values (Values): Current state estimates.
            jacobians (Optional[List[np.ndarray]]): Optional list to store Jacobian matrices.

        Returns:
            np.ndarray: Residual error array of shape (1,).
        """
        alpha = 30.0  # Controls the sharpness of the softmax approximation
        penalty_weight = 0.9  # Controls the strength of the soft constraint
        key1, key2 = this.keys()[0], this.keys()[1]
        pos1, pos2 = (
            values.atPose3(key1).translation(),
            values.atPose3(key2).translation(),
        )

        step_length = np.linalg.norm(pos1 - pos2)
        violation = step_length - measurement  # Constraint: step_length <= measurement

        # Softmax approximation of max(0, violation)
        softmax_penalty = (
            penalty_weight * (1 / alpha) * np.log(1 + np.exp(alpha * violation))
        )

        if jacobians is not None:
            if step_length > 1e-3:
                sigma_violation = 1 / (
                    1 + np.exp(-alpha * violation)
                )  # Corrected softmax derivative
                diff = (pos1 - pos2) / step_length  # Normalized direction vector
                J_pose = np.hstack(
                    [
                        penalty_weight * sigma_violation * diff.reshape(1, 3),
                        np.zeros((1, 3)),
                    ]
                )
                jacobians[0] = J_pose
                jacobians[1] = -J_pose
            else:
                epsilon = 0
                J_pose = np.hstack([np.full((1, 3), epsilon), np.zeros((1, 3))])
                jacobians[0] = J_pose
                jacobians[1] = -J_pose
        return np.array([softmax_penalty])

    def is_max_step_constraint_needed(
        self,
        predicted_state_right: PreintegratedImuMeasurements,
        predicted_state_left: PreintegratedImuMeasurements,
    ):
        pos_right_est = predicted_state_right.pose().translation()
        pos_left_est = predicted_state_left.pose().translation()

        step_length = np.linalg.norm(pos_right_est - pos_left_est)

        if step_length < self.max_foot_distance:
            return False
        else:
            return True

    def process_max_step_length_constraint(
        self, curr_pose_right_key: int, curr_pose_left_key: int
    ):
        step_length_factor = gtsam.CustomFactor(
            self.step_length_noise,
            [curr_pose_right_key, curr_pose_left_key],
            # Passes the error_step_length function with a fixed measurement value of 0.8 meters.
            partial(self.error_step_length, self.max_foot_distance),
        )
        self.new_factors.push_back(step_length_factor)
        self.insert_epoch_factor_dict(self.epoch_i)

    def process_zero_vel_measurements(
        self,
        curr_vel_key: int,
    ):
        """Adds a zero-velocity prior factor to the factor graph."""
        if not self.new_values.exists(curr_vel_key):
            KeyError(f"Key {curr_vel_key} does not exist in the current graph.")

        zero_vel_meas = np.zeros(3)
        self.new_factors.push_back(
            gtsam.PriorFactorVector(
                curr_vel_key,
                zero_vel_meas,
                self.zero_vel_noise,
            )
        )
        self.insert_epoch_factor_dict(self.epoch_i)

    def insert_epoch_factor_dict(self, epoch_i: int) -> None:
        self.curr_factor_index = self.curr_factor_index + 1
        self.epoch_factor_index_dict.setdefault(epoch_i, []).append(
            self.curr_factor_index
        )

    def retrieve_results_as_dateframe(self) -> pd.DataFrame:
        """
        Retrieve the results of the smoothing process as a pandas DataFrame.
        """
        results_dict = {
            "time_index": [],
            "pos_right_x": [],
            "pos_right_y": [],
            "pos_right_z": [],
            "pos_right_x_std": [],
            "pos_right_y_std": [],
            "pos_right_z_std": [],
            "pos_left_x": [],
            "pos_left_y": [],
            "pos_left_z": [],
            "pos_left_x_std": [],
            "pos_left_y_std": [],
            "pos_left_z_std": [],
            "vel_right_x": [],
            "vel_right_y": [],
            "vel_right_z": [],
            "vel_right_x_std": [],
            "vel_right_y_std": [],
            "vel_right_z_std": [],
            "vel_left_x": [],
            "vel_left_y": [],
            "vel_left_z": [],
            "vel_left_x_std": [],
            "vel_left_y_std": [],
            "vel_left_z_std": [],
            "acc_right_x": [],
            "acc_right_y": [],
            "acc_right_z": [],
            "acc_right_x_std": [],
            "acc_right_y_std": [],
            "acc_right_z_std": [],
            "acc_left_x": [],
            "acc_left_y": [],
            "acc_left_z": [],
            "acc_left_x_std": [],
            "acc_left_y_std": [],
            "acc_left_z_std": [],
            "gyro_right_x": [],
            "gyro_right_y": [],
            "gyro_right_z": [],
            "gyro_right_x_std": [],
            "gyro_right_y_std": [],
            "gyro_right_z_std": [],
            "gyro_left_x": [],
            "gyro_left_y": [],
            "gyro_left_z": [],
            "gyro_left_x_std": [],
            "gyro_left_y_std": [],
            "gyro_left_z_std": [],
        }

        time_indices = list(self.timeIndexToFirstPoseKey.keys())
        results_dict["time_index"] = time_indices

        for k in time_indices:
            pose_right_key = self.timeIndexToFirstPoseKey[k][0]
            pose_left_key = pose_right_key + 1
            vel_right_key = pose_right_key + 2
            vel_left_key = pose_right_key + 3
            bias_right_key = pose_right_key + 4
            bias_left_key = pose_right_key + 5

            if (
                self.gtsam_results.exists(pose_right_key)
                and self.gtsam_results.exists(pose_left_key)
                and self.gtsam_results.exists(bias_right_key)
                and self.gtsam_results.exists(bias_left_key)
            ):
                # Retrieve Pose3 translations
                pose_right = self.gtsam_results.atPose3(pose_right_key).translation()
                pose_left = self.gtsam_results.atPose3(pose_left_key).translation()

                vel_right = self.gtsam_results.atVector(vel_right_key)
                vel_left = self.gtsam_results.atVector(vel_left_key)

                # Retrieve Bias (IMU Bias contains both accelerometer & gyroscope biases)
                bias_right = self.gtsam_results.atConstantBias(bias_right_key)
                bias_left = self.gtsam_results.atConstantBias(bias_left_key)

                # Retrieve bias covariance
                if self.gtsam_optimizer_type == GtsamOptimizerType.LevenbergMarquard:
                    pos_right_cov = self.marginals.marginalCovariance(pose_right_key)
                    pos_left_cov = self.marginals.marginalCovariance(pose_left_key)
                    vel_right_cov = self.marginals.marginalCovariance(vel_right_key)
                    vel_left_cov = self.marginals.marginalCovariance(vel_left_key)
                    bias_cov_right = self.marginals.marginalCovariance(bias_right_key)
                    bias_cov_left = self.marginals.marginalCovariance(bias_left_key)
                else:
                    pos_right_cov = self.optimizer.marginalCovariance(pose_right_key)
                    pos_left_cov = self.optimizer.marginalCovariance(pose_left_key)
                    vel_right_cov = self.optimizer.marginalCovariance(vel_right_key)
                    vel_left_cov = self.optimizer.marginalCovariance(vel_left_key)
                    bias_cov_right = self.optimizer.marginalCovariance(bias_right_key)
                    bias_cov_left = self.optimizer.marginalCovariance(bias_left_key)

                # GTSAM results in body frame is z-axis up. Meas and GT are in z-axis down.
                results_dict["pos_right_x"].append(pose_right[0])
                results_dict["pos_right_y"].append(-pose_right[1])
                results_dict["pos_right_z"].append(-pose_right[2])
                results_dict["pos_right_x_std"].append(np.sqrt(pos_right_cov[0, 0]))
                results_dict["pos_right_y_std"].append(np.sqrt(pos_right_cov[1, 1]))
                results_dict["pos_right_z_std"].append(np.sqrt(pos_right_cov[2, 2]))
                results_dict["pos_left_x"].append(pose_left[0])
                results_dict["pos_left_y"].append(-pose_left[1])
                results_dict["pos_left_z"].append(-pose_left[2])
                results_dict["pos_left_x_std"].append(np.sqrt(pos_left_cov[0, 0]))
                results_dict["pos_left_y_std"].append(np.sqrt(pos_left_cov[1, 1]))
                results_dict["pos_left_z_std"].append(np.sqrt(pos_left_cov[2, 2]))
                results_dict["vel_right_x"].append(vel_right[0])
                results_dict["vel_right_y"].append(-vel_right[1])
                results_dict["vel_right_z"].append(-vel_right[2])
                results_dict["vel_right_x_std"].append(np.sqrt(vel_right_cov[0, 0]))
                results_dict["vel_right_y_std"].append(np.sqrt(vel_right_cov[1, 1]))
                results_dict["vel_right_z_std"].append(np.sqrt(vel_right_cov[2, 2]))
                results_dict["vel_left_x"].append(vel_left[0])
                results_dict["vel_left_y"].append(-vel_left[1])
                results_dict["vel_left_z"].append(-vel_left[2])
                results_dict["vel_left_x_std"].append(np.sqrt(vel_left_cov[0, 0]))
                results_dict["vel_left_y_std"].append(np.sqrt(vel_left_cov[1, 1]))
                results_dict["vel_left_z_std"].append(np.sqrt(vel_left_cov[2, 2]))
                results_dict["acc_right_x"].append(bias_right.accelerometer()[0])
                results_dict["acc_right_y"].append(bias_right.accelerometer()[1])
                results_dict["acc_right_z"].append(bias_right.accelerometer()[2])
                results_dict["acc_right_x_std"].append(np.sqrt(bias_cov_right[0, 0]))
                results_dict["acc_right_y_std"].append(np.sqrt(bias_cov_right[1, 1]))
                results_dict["acc_right_z_std"].append(np.sqrt(bias_cov_right[2, 2]))
                results_dict["acc_left_x"].append(bias_left.accelerometer()[0])
                results_dict["acc_left_y"].append(bias_left.accelerometer()[1])
                results_dict["acc_left_z"].append(bias_left.accelerometer()[2])
                results_dict["acc_left_x_std"].append(np.sqrt(bias_cov_left[0, 0]))
                results_dict["acc_left_y_std"].append(np.sqrt(bias_cov_left[1, 1]))
                results_dict["acc_left_z_std"].append(np.sqrt(bias_cov_left[2, 2]))
                results_dict["gyro_right_x"].append(bias_right.gyroscope()[0])
                results_dict["gyro_right_y"].append(bias_right.gyroscope()[1])
                results_dict["gyro_right_z"].append(bias_right.gyroscope()[2])
                results_dict["gyro_right_x_std"].append(np.sqrt(bias_cov_right[3, 3]))
                results_dict["gyro_right_y_std"].append(np.sqrt(bias_cov_right[4, 4]))
                results_dict["gyro_right_z_std"].append(np.sqrt(bias_cov_right[5, 5]))
                results_dict["gyro_left_x"].append(bias_left.gyroscope()[0])
                results_dict["gyro_left_y"].append(bias_left.gyroscope()[1])
                results_dict["gyro_left_z"].append(bias_left.gyroscope()[2])
                results_dict["gyro_left_x_std"].append(np.sqrt(bias_cov_left[3, 3]))
                results_dict["gyro_left_y_std"].append(np.sqrt(bias_cov_left[4, 4]))
                results_dict["gyro_left_z_std"].append(np.sqrt(bias_cov_left[5, 5]))

        return pd.DataFrame(results_dict)

    def retrieve_epoch_results(self, first_pose_key: int) -> OneEpochResult:
        pose_right = self.gtsam_results.atPose3(first_pose_key).translation()
        pose_left = self.gtsam_results.atPose3(first_pose_key + 1).translation()
        att_right = self.gtsam_results.atPose3(first_pose_key).rotation()
        att_left = self.gtsam_results.atPose3(first_pose_key + 1).rotation()
        vel_right = self.gtsam_results.atVector(first_pose_key + 2)
        vel_left = self.gtsam_results.atVector(first_pose_key + 3)
        bias_right = self.gtsam_results.atConstantBias(first_pose_key + 4)
        bias_left = self.gtsam_results.atConstantBias(first_pose_key + 5)
        if self.gtsam_optimizer_type == GtsamOptimizerType.LevenbergMarquard:
            pose_right_cov = self.marginals.marginalCovariance(first_pose_key)
            pose_left_cov = self.marginals.marginalCovariance(first_pose_key + 1)
            vel_right_cov = self.marginals.marginalCovariance(first_pose_key + 2)
            vel_left_cov = self.marginals.marginalCovariance(first_pose_key + 3)
            bias_right_cov = self.marginals.marginalCovariance(first_pose_key + 4)
            bias_left_cov = self.marginals.marginalCovariance(first_pose_key + 5)
        else:
            pose_right_cov = self.optimizer.marginalCovariance(first_pose_key)
            pose_left_cov = self.optimizer.marginalCovariance(first_pose_key + 1)
            vel_right_cov = self.optimizer.marginalCovariance(first_pose_key + 2)
            vel_left_cov = self.optimizer.marginalCovariance(first_pose_key + 3)
            bias_right_cov = self.optimizer.marginalCovariance(first_pose_key + 4)
            bias_left_cov = self.optimizer.marginalCovariance(first_pose_key + 5)
        return OneEpochResult(
            pose_right,
            pose_left,
            att_right,
            att_left,
            vel_right,
            vel_left,
            bias_right,
            bias_left,
            pose_right_cov,
            pose_left_cov,
            vel_right_cov,
            vel_left_cov,
            bias_right_cov,
            bias_left_cov,
        )

    def update_current_state(self):
        # Update the current state estimates for the next iteration
        if self.gtsam_results.exists(self.curr_pose_right_key):
            self.current_pose_right = self.gtsam_results.atPose3(
                self.curr_pose_right_key
            )
        if self.gtsam_results.exists(self.curr_pose_left_key):
            self.current_pose_left = self.gtsam_results.atPose3(self.curr_pose_left_key)

        if self.gtsam_results.exists(self.curr_vel_right_key):
            self.current_velocity_right = self.gtsam_results.atVector(
                self.curr_vel_right_key
            )
        if self.gtsam_results.exists(self.curr_vel_left_key):
            self.current_velocity_left = self.gtsam_results.atVector(
                self.curr_vel_left_key
            )

        if self.gtsam_results.exists(self.curr_bias_right_key):
            self.current_bias_right = self.gtsam_results.atConstantBias(
                self.curr_bias_right_key
            )
        if self.gtsam_results.exists(self.curr_bias_left_key):
            self.current_bias_left = self.gtsam_results.atConstantBias(
                self.curr_bias_left_key
            )

    def run_zero_vel_fgo(self):
        """
        Run the zero-velocity FGO estimator on the provided data.
        """

        # Preintegrators for each foot
        current_preint_right = PreintegratedImuMeasurements(
            self.gtsam_params_right, self.current_bias_right
        )
        current_preint_left = PreintegratedImuMeasurements(
            self.gtsam_params_left, self.current_bias_left
        )

        total_epochs = len(self.imu_right_pd)
        # Iterate over the data
        prev_zero_right_index = 0
        prev_zero_left_index = 0
        fixed_lag_head = 0
        epoch_count = 0

        pbar = tqdm(
            zip(self.imu_right_pd.itertuples(), self.imu_left_pd.itertuples()),
            total=total_epochs,
            desc="Processing",
            unit="epochs",
        )

        for k, (row_right, row_left) in enumerate(pbar):

            # Update the progress bar dynamically
            # pbar.set_postfix(
            #     factors_size=self.new_factors.size(),
            #     values_size=self.new_values.size(),
            #     epoch_factor_dict_size=len(self.epoch_factor_index_dict),
            # )

            if k == 0:
                continue  # Skip first iteration to ensure k-1 is valid

            # print(f"Right Bias at epoch {k}: {self.current_bias_right}")
            # print(f"Left Bias at epoch {k}: {self.current_bias_left}")

            prev_row_right = self.imu_right_pd.iloc[k - 1]  # Use IMU at k-1
            prev_row_left = self.imu_left_pd.iloc[k - 1]

            acc_right = np.array(
                [prev_row_right.acc_x, prev_row_right.acc_y, prev_row_right.acc_z]
            )
            gyro_right = np.array(
                [prev_row_right.gyro_x, prev_row_right.gyro_y, prev_row_right.gyro_z]
            )
            acc_left = np.array(
                [prev_row_left.acc_x, prev_row_left.acc_y, prev_row_left.acc_z]
            )
            gyro_left = np.array(
                [prev_row_left.gyro_x, prev_row_left.gyro_y, prev_row_left.gyro_z]
            )
            current_preint_right.integrateMeasurement(acc_right, gyro_right, self.dt)
            current_preint_left.integrateMeasurement(acc_left, gyro_left, self.dt)
            epoch_count = epoch_count + 1

            # check the predicted position, if the distance of
            # right and left is greater than 0.8m, then using a constraint measurement to update.
            # The nonlinear measurement model: norm(pose_left - pose_right) = 0.8

            # predicted_state_right = current_preint_right.predict(
            #     gtsam.NavState(self.current_pose_right, self.current_velocity_right),
            #     self.current_bias_right,
            # )
            # predicted_state_left = current_preint_left.predict(
            #     gtsam.NavState(self.current_pose_left, self.current_velocity_left),
            #     self.current_bias_left,
            # )
            # apply_step_length_constraint = self.is_max_step_constraint_needed(
            #     predicted_state_right, predicted_state_left
            # )

            # Determine if it is not a measurement epoch
            pos_match = self.position_meas_pd[
                self.position_meas_pd["time_index"] == row_right.time_index
            ]

            is_measurement_epoch = False
            is_valid_pos_meas = False
            is_valid_pos_meas = (
                not pos_match.empty
                and k % MeasurementParams.POS_MEAS_EPOCH_STEP == 1
                and k > 1
            ) and self.use_pos_meas
            is_measurement_epoch = is_measurement_epoch or is_valid_pos_meas

            is_valid_zero_vel_meas = False
            if (
                row_right.is_zero_vel == 1
                and self.use_zero_vel
                # and k - prev_zero_right_index >= MeasurementParams.ZUPT_EPOCH_STEP
            ):
                is_valid_zero_vel_meas = is_valid_zero_vel_meas or True
                prev_zero_right_index = k
            if (
                row_left.is_zero_vel == 1
                and self.use_zero_vel
                # and k - prev_zero_left_index >= MeasurementParams.ZUPT_EPOCH_STEP
            ):
                is_valid_zero_vel_meas = is_valid_zero_vel_meas or True
                prev_zero_left_index = k
            is_measurement_epoch = is_measurement_epoch or is_valid_zero_vel_meas

            if not self.use_pos_meas:
                is_valid_max_step_constraint = (
                    k % (MeasurementParams.MAX_FOOT_EPOCH_STEP) == 1 and k > 1
                ) and self.use_max_foot
            else:
                is_valid_max_step_constraint = (
                    k % MeasurementParams.MAX_FOOT_EPOCH_STEP == 1 and k > 1
                ) and self.use_max_foot
            if not self.use_pos_meas and not is_valid_zero_vel_meas:
                # If position measurement is not enabled.
                # Only apply the step length constraint when there is zero-velocity measurement
                is_valid_max_step_constraint = False
            is_measurement_epoch = is_measurement_epoch or is_valid_max_step_constraint

            if not is_measurement_epoch:
                continue

            # For each measurement epoch, update the epoch for factors and values
            self.prev_epoch_i = self.epoch_i
            self.epoch_i = k

            # Propagate new pose, velocity using IMU preintegration
            predicted_state_right = current_preint_right.predict(
                gtsam.NavState(self.current_pose_right, self.current_velocity_right),
                self.current_bias_right,
            )
            predicted_state_left = current_preint_left.predict(
                gtsam.NavState(self.current_pose_left, self.current_velocity_left),
                self.current_bias_left,
            )

            # apply_step_length_constraint = False
            # if is_posible_max_step_constraint:
            #     apply_step_length_constraint = self.is_max_step_constraint_needed(
            #         predicted_state_right, predicted_state_left
            #     )
            #     if not apply_step_length_constraint and not is_measurement_epoch:
            #         placeholder = 1
            #         continue
            # apply_step_length_constraint = False

            self.total_num_est_epochs = self.total_num_est_epochs + 1

            prev_pose_right_key = self.curr_pose_right_key
            prev_pose_left_key = self.curr_pose_left_key
            prev_vel_right_key = self.curr_vel_right_key
            prev_vel_left_key = self.curr_vel_left_key
            prev_bias_right_key = self.curr_bias_right_key
            prev_bias_left_key = self.curr_bias_left_key

            self.curr_pose_right_key = self.curr_pose_right_key + 6
            self.curr_pose_left_key = self.curr_pose_left_key + 6
            self.curr_vel_right_key = self.curr_vel_right_key + 6
            self.curr_vel_left_key = self.curr_vel_left_key + 6
            self.curr_bias_right_key = self.curr_bias_right_key + 6
            self.curr_bias_left_key = self.curr_bias_left_key + 6

            # Create IMU factors, add IMU factors linking (k-1) -> (k)
            self.new_factors.push_back(
                gtsam.ImuFactor(
                    prev_pose_right_key,
                    prev_vel_right_key,
                    self.curr_pose_right_key,
                    self.curr_vel_right_key,
                    prev_bias_right_key,
                    current_preint_right,
                )
            )
            self.insert_epoch_factor_dict(self.prev_epoch_i)

            self.new_factors.push_back(
                gtsam.ImuFactor(
                    prev_pose_left_key,
                    prev_vel_left_key,
                    self.curr_pose_left_key,
                    self.curr_vel_left_key,
                    prev_bias_left_key,
                    current_preint_left,
                )
            )
            self.insert_epoch_factor_dict(self.prev_epoch_i)

            # Add a between factor to constrain bias drift
            # Concatenate and scale std deviations (shape: (6,))
            bias_sigmas_right = np.sqrt(epoch_count) * np.concatenate(
                [
                    self.imu_params_right.acc_bias_std,
                    self.imu_params_right.gyro_bias_std,
                ]
            )
            self.bias_transition_noise_right = gtsam.noiseModel.Diagonal.Sigmas(
                bias_sigmas_right
            )
            bias_sigmas_left = np.sqrt(epoch_count) * np.concatenate(
                [
                    self.imu_params_left.acc_bias_std,
                    self.imu_params_left.gyro_bias_std,
                ]
            )
            self.bias_transition_noise_left = gtsam.noiseModel.Diagonal.Sigmas(
                bias_sigmas_left
            )
            # self.bias_transition_noise = gtsam.noiseModel.Diagonal.Sigmas(
            #     np.asarray(
            #         [np.sqrt(epoch_count) * self.bias_acc_sigma] * 3
            #         + [np.sqrt(epoch_count) * self.bias_gyro_sigma] * 3
            #     )
            # )

            # self.bias_transition_noise = gtsam.noiseModel.Isotropic.Sigma(6, 1e-2 * 2)
            self.new_factors.push_back(
                gtsam.BetweenFactorConstantBias(
                    prev_bias_right_key,  # Previous bias key
                    self.curr_bias_right_key,  # Current bias key
                    gtsam.imuBias.ConstantBias(),  # Expected bias difference (zero drift)
                    self.bias_transition_noise_right,  # Small uncertainty to allow slight variation
                )
            )
            self.insert_epoch_factor_dict(self.prev_epoch_i)

            self.new_factors.push_back(
                gtsam.BetweenFactorConstantBias(
                    prev_bias_left_key,
                    self.curr_bias_left_key,
                    gtsam.imuBias.ConstantBias(),
                    self.bias_transition_noise_left,
                )
            )
            self.insert_epoch_factor_dict(self.prev_epoch_i)

            # At each measurement epoch, reset the counter
            epoch_count = 0

            # Insert predicted states as initial estimates
            self.new_values.insert(
                self.curr_pose_right_key, predicted_state_right.pose()
            )
            self.new_values.insert(self.curr_pose_left_key, predicted_state_left.pose())
            self.new_values.insert(
                self.curr_vel_right_key, predicted_state_right.velocity()
            )
            self.new_values.insert(
                self.curr_vel_left_key, predicted_state_left.velocity()
            )
            self.new_values.insert(
                self.curr_bias_right_key, self.current_bias_right
            )  # Bias assumed constant
            self.new_values.insert(self.curr_bias_left_key, self.current_bias_left)

            self.epoch_key_dict.setdefault(self.epoch_i, []).extend(
                [
                    self.curr_pose_right_key,
                    self.curr_pose_left_key,
                    self.curr_vel_right_key,
                    self.curr_vel_left_key,
                    self.curr_bias_right_key,
                    self.curr_bias_left_key,
                ]
            )

            # Preintegrators for each foot
            current_preint_right = PreintegratedImuMeasurements(
                self.gtsam_params_right, self.current_bias_right
            )
            current_preint_left = PreintegratedImuMeasurements(
                self.gtsam_params_left, self.current_bias_left
            )

            # Position and velocity measurements update
            self.process_pos_measurements(
                k,
                is_valid_pos_meas,
                pos_match,
                self.curr_pose_right_key,
                self.curr_pose_left_key,
            )

            if is_valid_max_step_constraint:
                self.process_max_step_length_constraint(
                    self.curr_pose_right_key, self.curr_pose_left_key
                )

            if (
                self.use_zero_vel
                and row_right.is_zero_vel == 1
                and not is_valid_pos_meas
            ):
                self.process_zero_vel_measurements(self.curr_vel_right_key)
            if (
                self.use_zero_vel
                and row_left.is_zero_vel == 1
                and not is_valid_pos_meas
            ):
                self.process_zero_vel_measurements(self.curr_vel_left_key)

            if is_valid_pos_meas:
                meas_type = (
                    MeasType.Both
                    if (row_right.is_zero_vel == 1 or row_left.is_zero_vel == 1)
                    else MeasType.Position
                )
            else:
                meas_type = MeasType.ZeroVelocity

            self.timeIndexToFirstPoseKey[row_right.time_index] = (
                self.curr_pose_right_key,
                meas_type,
            )

            if self.gtsam_optimizer_type == GtsamOptimizerType.LevenbergMarquard:

                self.optimizer = gtsam.LevenbergMarquardtOptimizer(
                    self.new_factors, self.new_values
                )
                self.gtsam_results = self.optimizer.optimize()
                # Initialize marginals after optimization
                self.marginals = gtsam.Marginals(self.new_factors, self.gtsam_results)
            elif self.gtsam_optimizer_type == GtsamOptimizerType.ISAM2:
                self.optimizer.update(self.new_factors, self.new_values)
                self.new_factors = gtsam.NonlinearFactorGraph()
                # self.new_factors.resize(0)  # Clear the factor graph
                self.new_values = gtsam.Values()
                # Retrieve optimized estimates from ISAM2
                self.gtsam_results = self.optimizer.calculateEstimate()

            self.update_current_state()

        self.result_pd = self.retrieve_results_as_dateframe()
