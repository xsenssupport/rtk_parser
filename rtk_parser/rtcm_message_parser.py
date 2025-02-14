#!/usr/bin/env python3
from rtk_parser.gnss_tf import tf_wgs84_llh_ecef
import numpy as np

class RTCMMessageParser:
    @staticmethod
    def parse_1004(data):
        """Parse Extended L1&L2 GPS RTK Observables"""
        parsed = {
            "message_type": "1004",
            "description": "Extended L1&L2 GPS RTK Observables",
            "station_id": data.get("DF003", None),
            "gps_epoch_time": data.get("DF004", None),
            "num_satellites": data.get("NSat", 0),
            "satellites": []
        }
        
        for i in range(1, parsed["num_satellites"] + 1):
            sat = {
                "PRN": data.get(f"PRN_{i:02d}", None),
                "L1_code_ind": data.get(f"DF006_{i:02d}", None),
                "L1_pseudorange": data.get(f"DF007_{i:02d}", None),
                "L1_phase_range": data.get(f"DF008_{i:02d}", None),
                "L1_lock_time": data.get(f"DF009_{i:02d}", None),
                "L2_code_ind": data.get(f"DF010_{i:02d}", None),
                "L2_pseudorange": data.get(f"DF011_{i:02d}", None),
                "L2_phase_range": data.get(f"DF012_{i:02d}", None),
                "L2_lock_time": data.get(f"DF013_{i:02d}", None)
            }
            parsed["satellites"].append(sat)
        
        return parsed

    # @staticmethod
    # def parse_1005(data):
    #     """Parse Stationary RTK Reference Station ARP"""
    #     return {
    #         "message_type": "1005",
    #         "description": "Stationary RTK Reference Station ARP",
    #         "station_id": data.get("DF003", None),
    #         "position": {
    #             "x": data.get("DF025", None),
    #             "y": data.get("DF026", None),
    #             "z": data.get("DF027", None)
    #         },
    #         "indicator": data.get("DF024", None)
    #     }
    

    @staticmethod
    def parse_1005(data):
        """Parse Stationary RTK Reference Station ARP"""
        # Get ECEF coordinates
        x = data.get("DF025", None)
        y = data.get("DF026", None)
        z = data.get("DF027", None)

        # Create more informative output
        position_str = "=== RTCM Message ===\n"
        position_str += f"Type: 1005 - Stationary RTK Reference Station ARP\n"
        position_str += f"Station ID: {data.get('DF003', 'N/A')}\n\n"
        position_str += "Base Station Position:\n"
        
        if all(coord is not None for coord in [x, y, z]):
            ecef = np.array([x, y, z])
            lla = tf_wgs84_llh_ecef(ecef)
            lat_deg = np.degrees(lla[0])
            lon_deg = np.degrees(lla[1])
            alt_m = lla[2]
            
            position_str += f"  ECEF:\n"
            position_str += f"    X: {x:.4f} m\n"
            position_str += f"    Y: {y:.4f} m\n"
            position_str += f"    Z: {z:.4f} m\n\n"
            position_str += f"  LLA:\n"
            position_str += f"    Latitude:  {lat_deg:.8f}°\n"
            position_str += f"    Longitude: {lon_deg:.8f}°\n"
            position_str += f"    Altitude:  {alt_m:.4f} m\n"
        else:
            position_str += f"  X: {x if x is not None else 'N/A'}\n"
            position_str += f"  Y: {y if y is not None else 'N/A'}\n"
            position_str += f"  Z: {z if z is not None else 'N/A'}\n"

        # print(position_str)

        return {
            "message_type": "1005",
            "description": "Stationary RTK Reference Station ARP",
            "station_id": data.get("DF003", None),
            "position": {
                "ecef": {
                    "x": x,
                    "y": y,
                    "z": z
                },
                "lla": {
                    "lat": lat_deg if 'lat_deg' in locals() else None,
                    "lon": lon_deg if 'lon_deg' in locals() else None,
                    "alt": alt_m if 'alt_m' in locals() else None
                }
            },
            "indicator": data.get("DF024", None)
        }
    

    @staticmethod
    def parse_1006(data):
        """Parse Stationary RTK Reference Station ARP with Antenna Height"""
        parsed = RTCMMessageParser.parse_1005(data)
        parsed.update({
            "message_type": "1006",
            "description": "Stationary RTK Reference Station ARP with Antenna Height",
            "antenna_height": data.get("DF028", None)
        })
        return parsed


    @staticmethod
    def parse_1012(data):
        """Parse Extended L1&L2 GLONASS RTK Observables"""
        parsed = {
            "message_type": "1012",
            "description": "Extended L1&L2 GLONASS RTK Observables",
            "station_id": data.get("DF003", None),
            "glonass_epoch_time": data.get("DF034", None),
            "num_satellites": data.get("NSat", 0),
            "satellites": []
        }
        
        for i in range(1, parsed["num_satellites"] + 1):
            sat = {
                "PRN": data.get(f"PRN_{i:02d}", None),
                "L1_code_ind": data.get(f"DF035_{i:02d}", None),
                "L1_pseudorange": data.get(f"DF036_{i:02d}", None),
                "L1_phase_range": data.get(f"DF037_{i:02d}", None),
                "L1_lock_time": data.get(f"DF038_{i:02d}", None),
                "L2_code_ind": data.get(f"DF039_{i:02d}", None),
                "L2_pseudorange": data.get(f"DF040_{i:02d}", None),
                "L2_phase_range": data.get(f"DF041_{i:02d}", None),
                "L2_lock_time": data.get(f"DF042_{i:02d}", None)
            }
            parsed["satellites"].append(sat)
        
        return parsed


    @staticmethod
    def parse_1033(data):
        """Parse Receiver and Antenna Descriptors"""
        return {
            "message_type": "1033",
            "description": "Receiver and Antenna Descriptors",
            "station_id": data.get("DF003", None),
            "antenna_descriptor": data.get("DF226", ""),
            "antenna_serial": data.get("DF227", ""),
            "receiver_type": data.get("DF228", ""),
            "receiver_serial": data.get("DF229", ""),
            "receiver_firmware": data.get("DF230", "")
        }


    @staticmethod
    def parse_msm4(data, msg_type):
        """Parse MSM4 messages (1074, 1084, 1094, 1114, 1124)"""
        constellation_map = {
            "1074": "GPS",
            "1084": "GLONASS",
            "1094": "Galileo",
            "1124": "BeiDou"
        }
        
        constellation = constellation_map.get(msg_type, "Unknown")
        
        parsed = {
            "message_type": msg_type,
            "description": f"{constellation} MSM4",
            "constellation": constellation,
            "station_id": data.get("DF003", None),
            "epoch_time": data.get("DF004", None),
            "num_satellites": data.get("NSat", 0),
            "satellites": []
        }
        
        for i in range(1, parsed["num_satellites"] + 1):
            sat = {
                "PRN": f"{constellation[0]}{data.get(f'PRN_{i:02d}', None)}",  # Add constellation prefix
                "signal_strength": data.get(f"DF397_{i:02d}", None),
                "pseudorange": data.get(f"DF398_{i:02d}", None),
                "phase_range": data.get(f"DF399_{i:02d}", None),
                "lock_time": data.get(f"DF400_{i:02d}", None)
            }
            parsed["satellites"].append(sat)
        
        return parsed

    @staticmethod
    def parse_msm5(data, msg_type):
        """Parse MSM5 messages (1075, 1085, 1095, 1115, 1125)"""
        constellation_map = {
            "1075": "GPS",
            "1085": "GLONASS",
            "1095": "Galileo",
            "1125": "BeiDou"
        }
        
        constellation = constellation_map.get(msg_type, "Unknown")
        
        parsed = {
            "message_type": msg_type,
            "description": f"{constellation} MSM5",
            "constellation": constellation,
            "station_id": data.get("DF003", None),
            "epoch_time": data.get("DF004", None),
            "num_satellites": data.get("NSat", 0),
            "satellites": []
        }
        
        for i in range(1, parsed["num_satellites"] + 1):
            sat = {
                "PRN": f"{constellation[0]}{data.get(f'PRN_{i:02d}', None)}",  # Add constellation prefix
                # Extended signal attributes in MSM5
                "signal_strength": data.get(f"DF397_{i:02d}", None),  # CNR
                "pseudorange": data.get(f"DF398_{i:02d}", None),      # Fine Pseudorange
                "phase_range": data.get(f"DF399_{i:02d}", None),      # Fine PhaseRange
                "lock_time": data.get(f"DF400_{i:02d}", None),        # Lock Time Indicator
                "half_cycle_amb": data.get(f"DF401_{i:02d}", None),   # Half-cycle ambiguity indicator
                "cnr": data.get(f"DF402_{i:02d}", None),             # GNSS signal CNR
                "phase_range_rate": data.get(f"DF403_{i:02d}", None)  # PhaseRange Rate
            }
            parsed["satellites"].append(sat)
        
        return parsed

    @staticmethod
    def parse_message(msg_type, data):
        """Main parsing function"""
        parsing_functions = {
            "1004": RTCMMessageParser.parse_1004,
            "1005": RTCMMessageParser.parse_1005,
            "1006": RTCMMessageParser.parse_1006,
            "1012": RTCMMessageParser.parse_1012,
            "1033": RTCMMessageParser.parse_1033,
        }
        
        # MSM4 messages
        msm4_types = ["1074", "1084", "1094", "1124"]
        # MSM5 messages
        msm5_types = ["1075", "1085", "1095", "1125"]
        
        try:
            if msg_type in parsing_functions:
                return parsing_functions[msg_type](data)
            elif msg_type in msm4_types:
                return RTCMMessageParser.parse_msm4(data, msg_type)
            elif msg_type in msm5_types:
                return RTCMMessageParser.parse_msm5(data, msg_type)
            else:
                return {
                    "message_type": msg_type,
                    "description": "Unsupported message type",
                    "raw_data": data
                }
        except Exception as e:
            return {
                "message_type": msg_type,
                "description": "Error parsing message",
                "error": str(e),
                "raw_data": data
            }