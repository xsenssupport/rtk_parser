#!/usr/bin/env python3

import rclpy
import numpy as np
from geographiclib.geodesic import Geodesic
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticStatus
from rtk_interfaces.msg import BaselineStatus, GNSSStatus, Satellite

class RTKUtils:
    @staticmethod
    def calculate_baseline(base_pos: NavSatFix, rover_pos: NavSatFix) -> BaselineStatus:
        """Calculate baseline between base and rover positions."""
        baseline = BaselineStatus()
        baseline.header.stamp = rover_pos.header.stamp
        baseline.base_position = base_pos
        baseline.rover_position = rover_pos

        # Calculate geodetic distance using geographiclib
        geod = Geodesic.WGS84
        g = geod.Inverse(base_pos.latitude, base_pos.longitude,
                        rover_pos.latitude, rover_pos.longitude)
        
        baseline.distance_2d = g['s12']  # distance in meters
        height_diff = rover_pos.altitude - base_pos.altitude
        baseline.height_difference = height_diff
        baseline.distance_3d = np.sqrt(baseline.distance_2d**2 + height_diff**2)

        return baseline

    @staticmethod
    def create_gnss_status(satellite_data: dict, msg_ids: set) -> GNSSStatus:
        """Create GNSSStatus message from satellite data."""
        status = GNSSStatus()
        status.header.stamp = rclpy.time.Time().to_msg()
        status.satellite_count = len(satellite_data)
        
        # Safely handle SNR values, filtering out None values
        status.snr_values = [
            float(sat.get('signal_strength', 0.0)) 
            for sat in satellite_data.values() 
            if sat.get('signal_strength') is not None
        ]
        
        # Message IDs should already be integers
        status.rtcm_msg_ids = list(msg_ids)
        
        # Safely handle satellite systems
        status.satellite_systems = list(set(
            sat.get('system', 'Unknown') 
            for sat in satellite_data.values() 
            if sat.get('system') is not None
        ))
        status.coordinate_system = "WGS84"
        return status

    @staticmethod
    def create_satellite_msg(sat_data: dict) -> Satellite:
        """Create Satellite message from satellite data."""
        sat_msg = Satellite()
        sat_msg.header.stamp = rclpy.time.Time().to_msg()
        sat_msg.satellite_id = str(sat_data.get('PRN', ''))
        
        # Safely convert values with defaults
        sat_msg.carrier_phase = float(sat_data.get('phase_range', 0.0) or 0.0)
        sat_msg.pseudorange = float(sat_data.get('pseudorange', 0.0) or 0.0)
        sat_msg.doppler = float(sat_data.get('phase_range_rate', 0.0) or 0.0)
        sat_msg.snr = float(sat_data.get('signal_strength', 0.0) or 0.0)
        sat_msg.lock_time_indicator = int(sat_data.get('lock_time', 0) or 0)
        sat_msg.half_cycle_valid = bool(sat_data.get('half_cycle_amb', True))
        
        return sat_msg



    @staticmethod
    def get_rtk_quality_string(sat_count: int, avg_snr: float) -> str:
        """Determine RTK quality based on satellite count and signal strength."""
        if sat_count >= 8 and avg_snr >= 35:
            return "FIXED"
        elif sat_count >= 5 and avg_snr >= 25:
            return "FLOAT"
        else:
            return "NONE"