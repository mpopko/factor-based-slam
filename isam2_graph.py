import numpy as np
import gtsam
from gtsam.symbol_shorthand import B, V, X, L
from scipy.spatial.transform import Rotation


from utils import wrap_angle, unit3_from_angle, angles_from_unit3, is_visible

class ISAM2Graph:
    def __init__(self, prior_pose, bias, prior_vel, prior_std, prior_v_std, bias_cov, gyro_var, accel_var, g):
        """
        Initializes the iSAM2-based graph for state estimation with priors.
        
        Args:
            prior_pose (np.ndarray): Initial 4x4 pose matrix [m, rad].
            bias (gtsam.imuBias.ConstantBias): Initial IMU bias.
            prior_vel (np.ndarray): Initial velocity vector [m/s].
            prior_std (float): Standard deviation for pose prior.
            prior_v_std (float): Standard deviation for velocity prior.
            bias_cov (np.ndarray): Covariance for bias prior.
            gyro_var (float): Gyroscope noise variance [rad/s^2]^2.
            accel_var (float): Accelerometer noise variance [m/s^2]^2.
            g (float): Gravity magnitude [m/s^2] (UP convention).
        """
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()
        self.isam = self._init_isam()
        self.bias_index = 0
        self.state_index = 0

        self.bias = bias
        self.g = g

        self._add_priors(prior_pose, bias, prior_vel, prior_std, prior_v_std, bias_cov)
        self.pim = self._init_pim(bias, gyro_var, accel_var)

        self.current_estimate = None

        self.found_landmarks = set()

        self.x_smooth= np.array([prior_pose[:3, 3]])
        self.r_smooth = np.array([prior_pose[:3, :3]])
        self.v_smooth = np.array([prior_vel])
        self.landmark_positions = np.array([])
        self.b_smooth = np.array([bias])

        self.x_filter = [prior_pose[:3, 3]]
        self.r_filter = [prior_pose[:3, :3]]
        self.v_filter = [prior_vel]
        
        self.current_x_guess = gtsam.Pose3(prior_pose)


    def _init_isam(self):
        """
        Initializes the ISAM2 optimizer with default parameters.

        Returns:
            gtsam.ISAM2: The configured ISAM2 optimizer.
        """
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.0)
        params.relinearizeSkip = 1
        return gtsam.ISAM2(params)

    def _init_pim(self, bias, gyro_var, accel_var, int_std=1e-6):
        """
        Initializes IMU preintegration with noise parameters.

        Args:
            bias (gtsam.imuBias.ConstantBias): Initial bias.
            gyro_var (float): Gyroscope variance [rad^2/s^2].
            accel_var (float): Accelerometer variance [m^2/s^4].
            int_std (float): Integration noise standard deviation.

        Returns:
            gtsam.PreintegratedImuMeasurements: IMU preintegration object.
        """
        params = gtsam.PreintegrationParams.MakeSharedU(self.g)
        I = np.eye(3)
        params.setGyroscopeCovariance(gyro_var * I)
        params.setAccelerometerCovariance(accel_var * I)
        params.setIntegrationCovariance(int_std**2 * I)
        return gtsam.PreintegratedImuMeasurements(params, bias)

    def _add_priors(self, pose, bias, v0, pose_std, vel_std, bias_std):
        """
        Adds prior factors for pose, velocity, and bias to the factor graph.

        Args:
            pose (np.ndarray): Initial 4x4 pose matrix [m, rad].
            bias (gtsam.imuBias.ConstantBias): Initial bias.
            v0 (np.ndarray): Initial velocity [m/s].
            pose_std (float): Pose noise std dev.
            vel_std (float): Velocity noise std dev.
            bias_std (np.ndarray): Bias noise std dev.
        """
        noise_pose = gtsam.noiseModel.Isotropic.Sigma(6, pose_std)
        noise_vel = gtsam.noiseModel.Isotropic.Sigma(3, vel_std)
        noise_bias = gtsam.noiseModel.Diagonal.Sigmas(bias_std)

        pose3 = gtsam.Pose3(pose)
        self.graph.push_back(gtsam.PriorFactorPose3(X(0), pose3, noise_pose))
        self.graph.push_back(gtsam.PriorFactorVector(V(0), v0, noise_vel))
        self.graph.push_back(gtsam.PriorFactorConstantBias(B(0), bias, noise_bias))

        self.initial.insert(X(0), pose3)
        self.initial.insert(V(0), v0)
        self.initial.insert(B(0), bias)

    def preintegrate_imu(self, accel, gyro, dt):
        """
        Integrates IMU measurements for a time step.

        Args:
            accel (np.ndarray): Accelerometer reading [m/s^2].
            gyro (np.ndarray): Gyroscope reading [rad/s].
            dt (float): Time delta [s].
        """
        self.pim.integrateMeasurement(accel, gyro, dt)

    def add_imu_factor(self):
        """
        Adds a preintegrated IMU factor to the graph and predicts the next state.
        Updates internal pose and velocity guess.
        """
        i = self.state_index
        factor = gtsam.ImuFactor(X(i), V(i), X(i + 1), V(i + 1), B(self.bias_index), self.pim)
        self.graph.push_back(factor)
        x_guess = self.pim.predict(gtsam.NavState(
            self.current_x_guess,
            self.v_smooth[-1]
        ), self.b_smooth[-1])
        self.current_x_guess = x_guess.pose()
        self.initial.insert(X(i + 1), x_guess.pose())
        self.initial.insert(V(i + 1), x_guess.velocity())
        self.pim.resetIntegration()
        self.state_index += 1

    def add_landmark_obs(self, landmark_id, rel_bearing, rel_range, bearing_std, range_std):
        """
        Adds a landmark observation to the graph using bearing-range measurements.
        The landmark is conditioned on the current pose.

        Args:
            landmark_id (int): Unique ID of the landmark.
            rel_bearing (np.ndarray): Unit vector bearing from current pose.
            rel_range (float): Distance to landmark [m].
            bearing_std (float): Bearing angle std dev [deg].
            range_std (float): Range std dev [m].
        """
        noise_model = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([np.deg2rad(bearing_std), np.deg2rad(bearing_std), range_std]))
        self.graph.push_back(gtsam.BearingRangeFactor3D(X(self.state_index), L(landmark_id), gtsam.Unit3(rel_bearing), rel_range, noise_model))
        if landmark_id not in self.found_landmarks:
            self.initial.insert(L(landmark_id),
                                self.current_x_guess.rotation().matrix()@(rel_range*rel_bearing) + self.current_x_guess.translation())
            self.found_landmarks.add(landmark_id)

    def add_usbl_obs(self, position, noise_std):
        """
        Adds a 3D position observation (e.g., from USBL) as a GPSFactor.
        The usbl is conditioned on the current state.

        Args:
            position (np.ndarray): Observed position [m].
            noise_std (float): Standard deviation of the observation [m].
        """
        noise = gtsam.noiseModel.Isotropic.Sigma(3, noise_std)
        self.graph.push_back(gtsam.GPSFactor(X(self.state_index), position, noise))

    def update(self):
        """
        Inference/Optimization step.
        Updates the ISAM2 optimizer with current graph and values.
        Extracts the optimized state estimates.
        """
        self.isam.update(self.graph, self.initial)
        self.current_estimate = self.isam.calculateEstimate()
        self.initial.clear()
        self.graph = gtsam.NonlinearFactorGraph()
        self._extract_result()

    def advance_bias(self, bias_noise=0.1):
        """
        Advances to the next bias node with a between-factor to model bias evolution.

        Args:
            bias_noise (float): Bias noise std dev [rad/s, m/s^2].
        """
        noise = gtsam.noiseModel.Isotropic.Sigma(6, bias_noise)
        factor = gtsam.BetweenFactorConstantBias(B(self.bias_index), B(self.bias_index + 1), gtsam.imuBias.ConstantBias(), noise)
        self.graph.push_back(factor)
        self.initial.insert(B(self.bias_index + 1), self.current_estimate.atConstantBias(B(self.bias_index)))
        self.bias_index += 1
    
    def _extract_result(self):
        """
        Extracts the latest optimized states from ISAM2 estimate.
        Populates smoothed trajectories and landmark positions.
        """
        self.x_smooth = []
        self.r_smooth = []
        self.v_smooth = []
        self.b_smooth = []
        self.landmark_positions = []
        for i in range(self.state_index + 1):
            self.x_smooth.append(self.current_estimate.atPose3(X(i)).translation())
            self.r_smooth.append(self.current_estimate.atPose3(X(i)).rotation().matrix()[:3,:3])
            self.v_smooth.append(self.current_estimate.atVector(V(i)))
        for li in self.found_landmarks:
            self.landmark_positions.append(self.current_estimate.atPoint3(L(li)))
        for i in range(self.bias_index + 1):
            self.b_smooth.append(self.current_estimate.atConstantBias(B(i)))
        
        self.x_filter.append(self.current_estimate.atPose3(X(self.state_index)).translation())
        self.r_filter.append(self.current_estimate.atPose3(X(self.state_index)).rotation().matrix()[:3,:3])
        self.v_filter.append(self.current_estimate.atVector(V(self.state_index)))

    