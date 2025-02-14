#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RTCM
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64
import numpy as np
from pyrtcm import RTCMReader

from rtk_parser.rtcm_message_parser import RTCMMessageParser
from rtk_parser.rtk_utils import RTKUtils
from rtk_parser.rtk_diagnostics import RTKDiagnostics
from rtk_interfaces.msg import BaselineStatus, GNSSStatus, Satellite

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
        
        # Subscribers
        self.create_subscription(RTCM, '/rtcm', self.rtcm_callback, 10)
        self.create_subscription(Sentence, '/nmea', self.nmea_callback, 10)
        
        # State variables
        self.base_position = None
        self.rover_position = None
        self.satellite_data = {}
        self.rtcm_msg_types = set()
        self.receiver_info = None
        

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
        msg_type = parsed_data.get('message_type')
        
        if msg_type in ['1005', '1006']:
            self.handle_base_position(parsed_data)
        elif msg_type in ['1074', '1084', '1094', '1124', '1075', '1085', '1095', '1125']:
            self.handle_msm_message(parsed_data)
        elif msg_type in ['1033']:
            self.handle_receiver_info(parsed_data)
            
        self.publish_diagnostics()

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
        
        # Add satellite count diagnostic
        sat_count = len(self.satellite_data)
        level, message = RTKDiagnostics.check_satellite_count(sat_count)
        diag_array.status.append(
            RTKDiagnostics.create_diagnostic_status(
                "Satellite Count",
                level,
                message,
                {"count": sat_count}
            )
        )
        
        # Add signal strength diagnostic
        if self.satellite_data:
            avg_snr = np.mean([
                sat.get('signal_strength', 0.0) 
                for sat in self.satellite_data.values()
            ])
            level, message = RTKDiagnostics.check_snr(avg_snr)
            diag_array.status.append(
                RTKDiagnostics.create_diagnostic_status(
                    "Signal Strength",
                    level,
                    message,
                    {"average_snr": f"{avg_snr:.1f}"}
                )
            )
        
        self.pub_diagnostics.publish(diag_array)

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