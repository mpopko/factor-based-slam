import numpy as np
from scipy.spatial.transform import Rotation as R


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi

def angles_from_unit3(v):
    v = v / np.linalg.norm(v)
    x, y, z = v
    azimuth = np.degrees(np.arctan2(y, x))
    elevation = np.degrees(np.arcsin(z))
    return azimuth, elevation

def unit3_from_angle(azimuth, elevation):
    azimuth_rad = np.radians(azimuth)
    elevation_rad = np.radians(elevation)
    x = np.cos(elevation_rad) * np.cos(azimuth_rad)
    y = np.cos(elevation_rad) * np.sin(azimuth_rad)
    z = np.sin(elevation_rad)
    return np.array([x, y, z])

def is_visible(v, hor_range, vert_range):
    azimuth, elevation = angles_from_unit3(v)
    hor_min, hor_max = hor_range
    vert_min, vert_max = vert_range
    return hor_min <= azimuth <= hor_max and vert_min <= elevation <= vert_max


def remove_gravity(acc_list: np.ndarray, poses_list: np.ndarray, g=9.8) -> np.ndarray:
    R_world_from_imu = poses_list[:, :3, :3]

    rot_world_from_imu = R.from_matrix(R_world_from_imu)

    accel_world = rot_world_from_imu.apply(acc_list)

    gravity_world = np.array([0.0, 0.0, -g])
    accel_world_no_gravity = accel_world - gravity_world

    accel_body_no_gravity = rot_world_from_imu.inv().apply(accel_world_no_gravity)

    return accel_body_no_gravity