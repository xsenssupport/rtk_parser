#!/usr/bin/env python3

"""GNSS coordinate transformation functions.

This module provides functions for transforming between different GNSS coordinate systems
including ECEF, ENU, NED, and WGS84.

"""

import numpy as np
from typing import Tuple

class Constants:
    """WGS84 constants."""
    # Earth radius major axis [m]
    WGS84_A = 6378137.0
    # Earth radius minor axis [m]
    WGS84_B = 6356752.314245
    # 1/f inverse of flattening parameter
    WGS84_INV_F = 298.257223563
    # First eccentricity squared
    WGS84_E_2 = 6.69437999014e-3
    
    # Derived constants
    WGS84_A_2 = WGS84_A * WGS84_A
    WGS84_B_2 = WGS84_B * WGS84_B
    WGS84_E_PRIME_2 = WGS84_A_2 / WGS84_B_2 - 1

def rot_enu_ecef(lat: float, lon: float) -> np.ndarray:
    """Calculate rotation matrix from ECEF to ENU for given reference lat/lon.

    Args:
        lat: Reference latitude in radians
        lon: Reference longitude in radians

    Returns:
        3x3 rotation matrix from ECEF to ENU
    """
    s_lon = np.sin(lon)
    c_lon = np.cos(lon)
    s_lat = np.sin(lat)
    c_lat = np.cos(lat)
    
    rot = np.array([
        [-s_lon, c_lon, 0],
        [-c_lon * s_lat, -s_lon * s_lat, c_lat],
        [c_lon * c_lat, s_lon * c_lat, s_lat]
    ])
    return rot

def rot_ned_enu() -> np.ndarray:
    """Returns rotation matrix between NED and ENU.

    Returns:
        3x3 rotation matrix from NED to ENU
    """
    return np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1]
    ])

def rot_ned_ecef(lat: float, lon: float) -> np.ndarray:
    """Calculate rotation matrix from ECEF to NED.

    Args:
        lat: Reference latitude in radians
        lon: Reference longitude in radians

    Returns:
        3x3 rotation matrix from ECEF to NED
    """
    return rot_ned_enu() @ rot_enu_ecef(lat, lon)

def tf_ecef_wgs84_llh(wgs84llh: np.ndarray) -> np.ndarray:
    """Convert WGS84 (lat,lon,height) to ECEF coordinates.

    Args:
        wgs84llh: Array of [lat, lon, height] in [rad, rad, m]

    Returns:
        ECEF coordinates [x, y, z] in meters
    """
    lat, lon, height = wgs84llh
    s_lat = np.sin(lat)
    c_lat = np.cos(lat)
    s_lon = np.sin(lon)
    c_lon = np.cos(lon)
    
    # N is in meters
    n = Constants.WGS84_A / np.sqrt(1.0 - Constants.WGS84_E_2 * s_lat * s_lat)
    n_plus_height = n + height
    
    x = n_plus_height * c_lat * c_lon
    y = n_plus_height * c_lat * s_lon
    z = (n * (1 - Constants.WGS84_E_2) + height) * s_lat
    
    return np.array([x, y, z])

def tf_wgs84_llh_ecef(ecef: np.ndarray) -> np.ndarray:
    """Convert ECEF coordinates to WGS84 (lat,lon,height).

    Implementation based on Ferrari's solution.

    Args:
        ecef: ECEF coordinates [x, y, z] in meters

    Returns:
        WGS84 coordinates [lat, lon, height] in [rad, rad, m]
    """
    x, y, z = ecef
    
    x_2 = x * x
    y_2 = y * y
    z_2 = z * z
    r_2 = x_2 + y_2
    r = np.sqrt(r_2)
    
    F = 54.0 * Constants.WGS84_B_2 * z_2
    G = r_2 + (1 - Constants.WGS84_E_2) * z_2 - Constants.WGS84_E_2 * (
        Constants.WGS84_A_2 - Constants.WGS84_B_2)
    c = Constants.WGS84_E_2 * Constants.WGS84_E_2 * F * r_2 / (G * G * G)
    s = np.cbrt(1 + c + np.sqrt(c * c + 2 * c))
    P = F / (3.0 * (s + 1.0 + 1.0/s) * (s + 1.0 + 1.0/s) * G * G)
    Q = np.sqrt(1 + 2 * Constants.WGS84_E_2 * Constants.WGS84_E_2 * P)
    
    r0 = (-P * Constants.WGS84_E_2 * r / (1 + Q) + 
          np.sqrt(0.5 * Constants.WGS84_A_2 * (1.0 + 1.0/Q) -
                 P * (1 - Constants.WGS84_E_2) * z_2/(Q + Q * Q) - 0.5 * P * r_2))
    
    t1 = r - Constants.WGS84_E_2 * r0
    t1_2 = t1 * t1
    U = np.sqrt(t1_2 + z_2)
    V = np.sqrt(t1_2 + (1 - Constants.WGS84_E_2) * z_2)
    a_V = Constants.WGS84_A * V
    z0 = Constants.WGS84_B_2 * z / a_V
    
    height = U * (1 - Constants.WGS84_B_2 / a_V)
    lat = np.arctan((z + Constants.WGS84_E_PRIME_2 * z0) / r)
    lon = np.arctan2(y, x)
    
    return np.array([lat, lon, height])

def tf_enu_ecef(ecef: np.ndarray, wgs84llh_ref: np.ndarray) -> np.ndarray:
    """Transform ECEF coordinate to ENU with specified ENU-origin.

    Args:
        ecef: ECEF position to transform [x, y, z] in meters
        wgs84llh_ref: ENU-origin in [lat, lon, height] in [rad, rad, m]

    Returns:
        Position in ENU coordinates [e, n, u] in meters
    """
    ecef_ref = tf_ecef_wgs84_llh(wgs84llh_ref)
    return rot_enu_ecef(wgs84llh_ref[0], wgs84llh_ref[1]) @ (ecef - ecef_ref)

def quat_to_eul(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion to Euler angles (ZYX / yaw-pitch-roll).

    Args:
        quat: Quaternion [w, x, y, z]

    Returns:
        Euler angles [yaw, pitch, roll] in radians
    """
    qw, qx, qy, qz = quat
    
    yaw = np.arctan2(2 * (qx * qy + qw * qz),
                     qw * qw + qx * qx - qy * qy - qz * qz)
    pitch = np.arcsin(-2.0 * (qx * qz - qw * qy))
    roll = np.arctan2(2 * (qy * qz + qw * qx),
                      qw * qw - qx * qx - qy * qy + qz * qz)
    
    return np.array([yaw, pitch, roll])