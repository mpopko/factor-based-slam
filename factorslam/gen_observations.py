# observation_utils.py
import numpy as np

from utils import angles_from_unit3, is_visible, unit3_from_angle

def generate_usbl_data(traj, noise_std=0.1, rate=1, seed=42):
    """
    Simulates noisy USBL (position) observations from a trajectory.

    Args:
        traj (list or np.ndarray): List of 3D positions [m].
        noise_std (float): Standard deviation of positional noise [m].
        rate (int): Sampling rate (take every `rate` steps).
        seed (int): Random seed for reproducibility.

    Returns:
        dict: Mapping from timestep index to noisy position [m].
    """
    np.random.seed(seed)
    usbl_obs = []
    for i in range(0, len(traj), rate):
        noisy_pos = traj[i] + np.random.normal(0, noise_std, size=3)
        usbl_obs.append((i, noisy_pos))
    return dict(usbl_obs)

def generate_landmark_observations(traj, landmarks, fov_h=(-45,45), fov_v=(-50,10), min_range=0.5, max_range=12,
                                    bearing_noise=1.0, range_noise=0.1, seed=42):
    """
    Simulates landmark observations with noise and visibility constraints.

    Args:
        traj (list or np.ndarray): List of 4x4 pose matrices.
        landmarks (np.ndarray): Nx3 array of landmark positions [m].
        fov_h (tuple): Horizontal field of view in degrees (min, max).
        fov_v (tuple): Vertical field of view in degrees (min, max).
        min_range (float): Minimum range to observe a landmark [m].
        max_range (float): Maximum range to observe a landmark [m].
        bearing_noise (float): Bearing angle noise [deg].
        range_noise (float): Range noise [m].
        seed (int): Random seed for reproducibility.

    Returns:
        dict: Mapping from timestep index to {landmark_id: [bearing (Unit3), range]}.
    """
    np.random.seed(seed)
    observations = {}
    for i, pose in enumerate(traj):
        pose3 = pose[:3, :3], pose[:3, 3]
        for li, lm in enumerate(landmarks):
            rel = lm - pose3[1]
            rel_local = pose3[0].T @ rel
            if is_visible(rel_local, fov_h, fov_v):
                dist = np.linalg.norm(rel_local)
                if min_range < dist < max_range:
                    az, el = angles_from_unit3(rel_local)
                    az += np.random.normal(0, bearing_noise)
                    el += np.random.normal(0, bearing_noise)
                    bearing = unit3_from_angle(az, el)
                    dist += np.random.normal(0, range_noise)
                    if i in observations.keys():
                        observations[i][li] = [bearing, dist]
                    else:
                        observations[i] = {li: [bearing, dist]}
    return observations
