#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RTCM
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from rtk_interfaces.msg import RTKAnalysis
from std_msgs.msg import Float64
import numpy as np
from pyrtcm import RTCMReader
import json
from datetime import datetime

from rtk_parser.rtcm_message_parser import RTCMMessageParser
from rtk_parser.rtk_utils import RTKUtils
from rtk_parser.rtk_diagnostics import RTKDiagnostics
from rtk_interfaces.msg import BaselineStatus, GNSSStatus, Satellite
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue


class RTKParser(Node):
    def __init__(self):
        super().__init__('rtk_parser_node')
        
        # Initialize RTCM reader
        self.stream = RTCMReader(None)
        
        # Publishers
        self.pub_base_pos = self.create_publisher(NavSatFix, '/rtcm_parser/base_position', 10)
        self.pub_rover_pos = self.create_publisher(NavSatFix, 'rtcm_parser/rover_position', 10)
        self.pub_baseline = self.create_publisher(BaselineStatus, '/rtcm_parser/baseline_status', 10)
        self.pub_gnss_status = self.create_publisher(GNSSStatus, '/rtcm_parser/gnss_status', 10)
        self.pub_satellite = self.create_publisher(Satellite, '/rtcm_parser/satellite_info', 10)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray, '/rtcm_parser/rtk_diagnostics', 10)
        self.pub_analysis = self.create_publisher(RTKAnalysis, '/rtcm_parser/rtk_analysis', 10)
        
        # Subscribers
        self.create_subscription(RTCM, '/rtcm', self.rtcm_callback, 10)
        self.create_subscription(Sentence, '/nmea', self.nmea_callback, 10)
        
        # State variables
        self.base_position = None
        self.rover_position = None
        self.satellite_data = {}
        self.rtcm_msg_types = set()
        self.receiver_info = None
        
        self.parsed_messages = []
        self.max_messages = 1000  # Limit to prevent memory overflow
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(1.0, self.perform_analysis)  # Run every 1 second
        
        self.get_logger().info("RTK Parser node initialized")

    def nmea_callback(self, msg: Sentence):
        """Process NMEA messages from rover."""
        try:
            if msg.sentence.startswith('$GPGGA'):
                pos = self.parse_nmea_gga(msg.sentence)
                if pos is not None:
                    self.rover_position = pos
                    self.pub_rover_pos.publish(pos)
                    self.update_baseline()
        except Exception as e:
            self.get_logger().error(f"Error in NMEA callback: {str(e)}")

    def rtcm_callback(self, msg: RTCM):
        """Process RTCM messages."""
        try:
            rtcm_bytes = bytes(msg.data)
            parsed_msg = self.stream.parse(rtcm_bytes)
            
            if parsed_msg:
                parsed_data = RTCMMessageParser.parse_message(
                    parsed_msg.identity, 
                    parsed_msg.__dict__
                )
                
                # Convert message ID to integer and add to set
                try:
                    # Handle both string and integer cases
                    if isinstance(parsed_msg.identity, str):
                        msg_id = int(parsed_msg.identity)
                    else:
                        msg_id = parsed_msg.identity
                        
                    if 0 <= msg_id <= 65535:  # Ensure it's within uint16 range
                        self.rtcm_msg_types.add(msg_id)
                    else:
                        self.get_logger().warning(f"RTCM message ID out of range: {msg_id}")
                except (ValueError, TypeError) as e:
                    self.get_logger().warning(f"Invalid RTCM message ID: {parsed_msg.identity}")
                
                # Process different message types
                self.process_rtcm_message(parsed_data)
                
        except Exception as e:
            self.get_logger().error(f"Error in RTCM callback: {str(e)}")

    def process_rtcm_message(self, parsed_data: dict):
            """Process parsed RTCM message based on type."""
            # Add timestamp and store the message
            parsed_data['timestamp'] = datetime.now().isoformat()
            self.parsed_messages.append(parsed_data)
            if len(self.parsed_messages) > self.max_messages:
                self.parsed_messages.pop(0)
            
            msg_type = parsed_data.get('message_type')
            
            if msg_type in ['1005', '1006']:
                self.handle_base_position(parsed_data)
            elif msg_type in ['1074', '1084', '1094', '1124', '1075', '1085', '1095', '1125']:
                self.handle_msm_message(parsed_data)
            elif msg_type in ['1033']:
                self.handle_receiver_info(parsed_data)
            elif msg_type in ['1230']:
                self.handle_glonass_biases(parsed_data)
                
            self.publish_diagnostics()

    def handle_glonass_biases(self, data: dict):
            """Process GLONASS bias messages."""
            self.get_logger().info(f"GLONASS Biases: {data.get('biases', {})}")

    def parse_nmea_gga(self, sentence: str) -> NavSatFix:
        """
        Parse NMEA GGA sentence and return NavSatFix message.
        
        $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        
        Fields:
        0 = Message ID ($GPGGA)
        1 = UTC Time (HHMMSS.ss)
        2 = Latitude (DDMM.mmm)
        3 = N/S Indicator
        4 = Longitude (DDDMM.mmm)
        5 = E/W Indicator
        6 = GPS Quality (0=No fix, 1=GPS fix, 2=DGPS fix)
        7 = Number of satellites
        8 = HDOP
        9 = Altitude
        10 = Altitude units (M=meters)
        11 = Geoidal separation
        12 = Separation units
        13 = Age of differential correction
        14 = Checksum
        """
        try:
            fields = sentence.split(',')
            if len(fields) < 15:
                self.get_logger().warning("Invalid GGA sentence: insufficient fields")
                return None

            # Parse latitude
            if fields[2] and fields[3]:
                lat_deg = float(fields[2][:2])
                lat_min = float(fields[2][2:])
                latitude = lat_deg + lat_min/60.0
                if fields[3] == 'S':
                    latitude = -latitude

            # Parse longitude
            if fields[4] and fields[5]:
                lon_deg = float(fields[4][:3])
                lon_min = float(fields[4][3:])
                longitude = lon_deg + lon_min/60.0
                if fields[5] == 'W':
                    longitude = -longitude

            # Parse altitude
            altitude = float(fields[9]) if fields[9] else 0.0

            # Create NavSatFix message
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = "gps"
            fix.latitude = latitude
            fix.longitude = longitude
            fix.altitude = altitude

            # Set covariance based on GPS quality indicator
            quality = int(fields[6]) if fields[6] else 0
            if quality == 4:  # RTK fixed
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                fix.position_covariance = [0.0001, 0.0, 0.0,  # ~1cm
                                        0.0, 0.0001, 0.0,
                                        0.0, 0.0, 0.0001]
            elif quality == 5:  # RTK float
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                fix.position_covariance = [0.01, 0.0, 0.0,    # ~10cm
                                        0.0, 0.01, 0.0,
                                        0.0, 0.0, 0.01]
            elif quality in [1, 2]:  # GPS or DGPS
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                fix.position_covariance = [1.0, 0.0, 0.0,     # ~1m
                                        0.0, 1.0, 0.0,
                                        0.0, 0.0, 1.0]
            else:  # Unknown or no fix
                fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            return fix

        except Exception as e:
            self.get_logger().error(f"Error parsing GGA sentence: {str(e)}")
            return None
    

    def handle_receiver_info(self, data: dict):
        """Process receiver and antenna information messages."""
        try:
            self.get_logger().info(
                f"Receiver Info:\n"
                f"  Station ID: {data.get('station_id', 'N/A')}\n"
                f"  Antenna: {data.get('antenna_descriptor', 'N/A')} "
                f"(S/N: {data.get('antenna_serial', 'N/A')})\n"
                f"  Receiver: {data.get('receiver_type', 'N/A')} "
                f"(S/N: {data.get('receiver_serial', 'N/A')})\n"
                f"  Firmware: {data.get('receiver_firmware', 'N/A')}"
            )
            
            # You could store this information as class attributes if needed
            self.receiver_info = {
                'station_id': data.get('station_id'),
                'antenna_descriptor': data.get('antenna_descriptor'),
                'antenna_serial': data.get('antenna_serial'),
                'receiver_type': data.get('receiver_type'),
                'receiver_serial': data.get('receiver_serial'),
                'receiver_firmware': data.get('receiver_firmware')
            }
            
        except Exception as e:
            self.get_logger().error(f"Error handling receiver info: {str(e)}")



    def handle_base_position(self, data: dict):
        """Process base station position messages."""
        if 'position' in data:
            pos = NavSatFix()
            pos.header.stamp = self.get_clock().now().to_msg()
            pos.header.frame_id = "base_station"
            
            lla = data['position']['lla']
            pos.latitude = lla['lat']
            pos.longitude = lla['lon']
            pos.altitude = lla['alt']
            
            self.base_position = pos
            self.pub_base_pos.publish(pos)
            self.update_baseline()

    def handle_msm_message(self, data: dict):
        """Process MSM messages for satellite information."""
        if 'satellites' in data:
            for sat in data['satellites']:
                if sat.get('PRN') is not None:  # Only process if we have a valid PRN
                    sat['system'] = data.get('constellation', 'Unknown')
                    self.satellite_data[sat['PRN']] = sat
            
            # Update GNSS status
            status = RTKUtils.create_gnss_status(
                self.satellite_data,
                list(self.rtcm_msg_types)
            )
            self.pub_gnss_status.publish(status)
            
            # Publish individual satellite information
            for sat_data in data['satellites']:
                if sat_data.get('PRN') is not None:  # Only process if we have a valid PRN
                    sat_msg = RTKUtils.create_satellite_msg(sat_data)
                    self.pub_satellite.publish(sat_msg)

    def update_baseline(self):
        """Update and publish baseline information."""
        if self.base_position and self.rover_position:
            baseline = RTKUtils.calculate_baseline(
                self.base_position,
                self.rover_position
            )
            
            # Update satellite count and average SNR
            if self.satellite_data:
                baseline.satellite_count = len(self.satellite_data)
                
                # Safely calculate average SNR
                valid_snr_values = [
                    float(sat.get('signal_strength', 0.0))
                    for sat in self.satellite_data.values()
                    if sat.get('signal_strength') is not None
                ]
                
                if valid_snr_values:
                    baseline.average_snr = float(np.mean(valid_snr_values))
                else:
                    baseline.average_snr = 0.0
                    
                baseline.rtk_status = RTKUtils.get_rtk_quality_string(
                    baseline.satellite_count,
                    baseline.average_snr
                )
                
                self.pub_baseline.publish(baseline)

    def publish_diagnostics(self):
        """Publish diagnostic information."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Satellite count
        sat_count = len(self.satellite_data)
        level, message = RTKDiagnostics.check_satellite_count(sat_count)
        diag_array.status.append(
            RTKDiagnostics.create_diagnostic_status(
                "Satellite Count", level, message, {"count": sat_count}
            )
        )
        
        # Signal strength
        if self.satellite_data:
            avg_snr = np.mean([sat.get('signal_strength', 0.0) for sat in self.satellite_data.values()])
            level, message = RTKDiagnostics.check_snr(avg_snr)
            diag_array.status.append(
                RTKDiagnostics.create_diagnostic_status(
                    "Signal Strength", level, message, {"average_snr": f"{avg_snr:.1f}"}
                )
            )
        
        # Critical messages check
        stats = self.calculate_message_statistics()
        if stats["critical_messages_check"]["missing_1005_1006"]:
            diag_array.status.append(
                RTKDiagnostics.create_diagnostic_status(
                    "Critical Messages",
                    DiagnosticStatus.WARN,
                    "Missing base station position (1005/1006)",
                    {}
                )
            )
        if stats["critical_messages_check"]["missing_1033"]:
            diag_array.status.append(
                RTKDiagnostics.create_diagnostic_status(
                    "Critical Messages",
                    DiagnosticStatus.WARN,
                    "Missing receiver/antenna info (1033)",
                    {}
                )
            )
        
        # Signal quality per system
        signal_quality = self.analyze_signal_quality()
        for system, quality in signal_quality.items():
            level = DiagnosticStatus.OK if quality["quality_assessment"] == "Good" else \
                   DiagnosticStatus.WARN if quality["quality_assessment"] == "Marginal" else \
                   DiagnosticStatus.ERROR
            diag_array.status.append(
                RTKDiagnostics.create_diagnostic_status(
                    f"{system.upper()} Signal Quality",
                    level,
                    f"{quality['quality_assessment']} signal quality",
                    {"average_snr": f"{quality['average_snr']:.1f}"}
                )
            )
        
        self.pub_diagnostics.publish(diag_array)


    def calculate_message_statistics(self):
        """Calculate message statistics."""
        message_counts = {}
        for msg in self.parsed_messages:
            msg_type = msg.get("message_type")
            if msg_type:
                message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
        
        total_messages = len(self.parsed_messages)
        time_span = {
            "start": min((msg.get("timestamp", "") for msg in self.parsed_messages), default=""),
            "end": max((msg.get("timestamp", "") for msg in self.parsed_messages), default="")
        }
        critical_messages_check = {
            "missing_1005_1006": not any(msg.get("message_type") in ["1005", "1006"] for msg in self.parsed_messages),
            "missing_1033": not any(msg.get("message_type") == "1033" for msg in self.parsed_messages)
        }
        
        return {
            "total_messages": total_messages,
            "message_counts": message_counts,
            "time_span": time_span,
            "critical_messages_check": critical_messages_check
        }

    def analyze_signal_quality(self):
        """Analyze signal quality per GNSS system."""
        msm_types = {
            "gps": ["1074", "1075", "1077"],
            "glonass": ["1084", "1085", "1087"],
            "galileo": ["1094", "1095", "1097"],
            "beidou": ["1124", "1125", "1127"]
        }
        
        signal_quality = {}
        
        for system, msg_types in msm_types.items():
            satellites = []
            for msg in self.parsed_messages:
                if msg.get("message_type") in msg_types:
                    for sat in msg.get("satellites", []):
                        if sat.get("PRN") and sat.get("signal_strength") is not None:
                            satellites.append({
                                "PRN": sat["PRN"],
                                "SNR": float(sat["signal_strength"]),
                                "quality": "Good" if sat["signal_strength"] > 40 else "Marginal" if sat["signal_strength"] > 30 else "Poor"
                            })
            
            if satellites:
                avg_snr = sum(sat["SNR"] for sat in satellites) / len(satellites)
                signal_quality[system] = {
                    "average_snr": avg_snr,
                    "quality_assessment": "Good" if avg_snr > 40 else "Marginal" if avg_snr > 30 else "Poor"
                }
        
        return signal_quality

    def get_most_recent_message(self, msg_types):
        """Get the most recent message from a list of message types."""
        candidates = [msg for msg in self.parsed_messages if msg.get("message_type") in msg_types]
        if candidates:
            return max(candidates, key=lambda x: x["timestamp"])
        return None

    def analyze_overall(self):
        """Perform overall RTK analysis using the most recent messages."""
        # Define MSM message types for each GNSS system
        msm_types = {
            "gps": ["1074", "1075", "1077"],
            "glonass": ["1084", "1085", "1087"],
            "galileo": ["1094", "1095", "1097"],
            "beidou": ["1124", "1125", "1127"]
        }

        # Track unique satellites and SNR values
        unique_satellites = set()  # Use a set to ensure no duplicates
        total_snr_sum = 0.0
        total_sats_for_snr = 0
        sat_info = {}

        # Process the most recent message for each system
        for system, msg_types in msm_types.items():
            recent_msg = self.get_most_recent_message(msg_types)
            if recent_msg:
                satellites = recent_msg.get("satellites", [])
                # Collect unique PRNs from this message
                for sat in satellites:
                    prn = sat.get("PRN")
                    if prn:
                        unique_satellites.add(f"{system}_{prn}")  # e.g., "gps_G01", "glonass_R01"

                # Store satellite count for this system
                sat_info[system] = {"num_satellites": len(satellites)}

                # Calculate SNR for quality assessment
                snr_values = [sat.get("signal_strength", 0.0) for sat in satellites if sat.get("signal_strength") is not None]
                if snr_values:
                    total_snr_sum += sum(snr_values)
                    total_sats_for_snr += len(snr_values)

        # Total satellites is the number of unique satellites across all systems
        total_sats = len(unique_satellites)
        avg_snr_all = total_snr_sum / total_sats_for_snr if total_sats_for_snr > 0 else 0.0

        # Compile overall analysis
        overall = {
            "total_satellites": total_sats,
            "average_snr_all_systems": avg_snr_all,
            "rtk_fix_likelihood": "High" if avg_snr_all > 40 and total_sats >= 10 else
                                 "Medium" if avg_snr_all > 30 and total_sats >= 8 else "Low",
            "multi_gnss_usage": len([sys for sys in sat_info if sat_info[sys]["num_satellites"] > 0]),
            "potential_issues": [],
            "recommendations": []
        }

        # Add potential issues and recommendations
        if avg_snr_all < 35:
            overall["potential_issues"].append("Low overall signal strength")
            overall["recommendations"].append("Check sky view and reduce interference")
        if total_sats < 8:
            overall["potential_issues"].append("Low satellite count")
            overall["recommendations"].append("Wait for better satellite visibility")

        return overall


    def perform_analysis(self):
        """Perform and publish all analyses."""
        stats = self.calculate_message_statistics()
        signal_quality = self.analyze_signal_quality()
        overall = self.analyze_overall()
        
        # Compile base station info
        base_info = next((msg for msg in self.parsed_messages if msg.get("message_type") in ["1005", "1006"]), None)
        base_station_info = base_info if base_info else {}
        
        # Compile antenna/receiver info
        antenna_info = next((msg for msg in self.parsed_messages if msg.get("message_type") == "1033"), None) or {}
        
        # Compile GLONASS biases
        glonass_biases = next((msg for msg in self.parsed_messages if msg.get("message_type") == "1230"), None) or {}
        
        # Compile satellite info for detailed output (optional)
        msm_types = {
            "gps": ["1074", "1075", "1077"],
            "glonass": ["1084", "1085", "1087"],
            "galileo": ["1094", "1095", "1097"],
            "beidou": ["1124", "1125", "1127"]
        }
        # sat_info = {}
        # for system, msg_types in msm_types.items():
        #     recent_msg = self.get_most_recent_message(msg_types)
        #     if recent_msg:
        #         satellites = recent_msg.get("satellites", [])
        #         sat_info[system] = {
        #             "num_satellites": len(satellites),
        #             "satellites": satellites
        #         }
        
        # Create and publish RTKAnalysis message
        analysis_msg = RTKAnalysis()
        analysis_msg.header.stamp = self.get_clock().now().to_msg()
        analysis_msg.timestamp = datetime.now().isoformat()
        analysis_msg.total_messages = stats["total_messages"]
        analysis_msg.message_counts = json.dumps(stats["message_counts"])
        analysis_msg.time_span = json.dumps(stats["time_span"])
        analysis_msg.critical_messages_check = json.dumps(stats["critical_messages_check"])
        analysis_msg.base_station_info = json.dumps(base_station_info)
        analysis_msg.antenna_info = json.dumps(antenna_info)
        analysis_msg.signal_quality = json.dumps(signal_quality)
        # analysis_msg.satellite_info = json.dumps(sat_info)
        analysis_msg.glonass_biases = json.dumps(glonass_biases)
        analysis_msg.overall_analysis = json.dumps(overall)
        
        self.pub_analysis.publish(analysis_msg)



def main(args=None):
    rclpy.init(args=args)
    node = RTKParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()