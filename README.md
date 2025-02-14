# RTK Parser ROS2 Package

## Overview
RTK Parser is a ROS2 package designed to parse and analyze RTCM messages for RTK (Real-Time Kinematic) positioning systems. It provides comprehensive analysis of RTCM data, baseline distance calculations, signal quality monitoring, and satellite tracking capabilities.

### Features
- Parse various RTCM message types (1001-1230)
- Calculate baseline distance between base station and rover
- Monitor satellite signal quality and count
- Provide diagnostic information
- Support multiple GNSS constellations (GPS, GLONASS, Galileo, BeiDou)
- Real-time position tracking for both base station and rover

### Supported RTCM Messages
| Message Type | Description |
|-------------|-------------|
| 1001-1004   | GPS RTK Observables |
| 1005-1006   | Station Coordinates |
| 1007        | Antenna Descriptor |
| 1009-1012   | GLONASS RTK Observables |
| 1033        | Receiver and Antenna Description |
| 1074-1077   | GPS MSM4-7 |
| 1084-1087   | GLONASS MSM4-7 |
| 1094-1097   | Galileo MSM4-7 |
| 1124-1127   | BeiDou MSM4-7 |
| 1230        | GLONASS Code-Phase Biases |

## Installation

### Prerequisites
- ROS2 (Tested on Humble)
- Python 3.8+
- pip
- [rtk_interfaces](https://github.com/xsenssupport/rtk_interfaces) - Custom message definitions for RTK data

### Dependencies
```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-numpy \
    ros-humble-diagnostic-msgs \
    ros-humble-mavros-msgs \
    ros-humble-nmea-msgs

pip3 install pyrtcm geographiclib
```

### Building from Source
```bash
# Create a workspace (if you don't have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# First, clone and build rtk_interfaces
git clone https://github.com/yourusername/rtk_interfaces.git

# Then clone this repository
git clone https://github.com/yourusername/rtk_parser.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the packages (order matters)
colcon build --packages-select rtk_interfaces
colcon build --packages-select rtk_parser

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### Launch the Node
```bash
ros2 launch rtk_parser rtk_parser.launch.py
```

With debug output enabled:
```bash
ros2 launch rtk_parser rtk_parser.launch.py debug:=true
```

### Published Topics
| Topic Name         | Message Type | Description |
|--------------------|-----------------------------------|-----------------------|
| `/rtcm_parser/base_position`   | `sensor_msgs/NavSatFix`           | Base station position |
| `/rtcm_parser/rover_position`  | `sensor_msgs/NavSatFix`           | Rover position        |
| `/rtcm_parser/baseline_status` | `rtk_parser/BaselineStatus`       | Baseline information  |
| `/rtcm_parser/gnss_status`     | `rtk_parser/GNSSStatus`           | GNSS system status    |
| `/rtcm_parser/satellite_info`  | `rtk_parser/Satellite`            | Individual satellite information |
| `/rtcm_parser/rtk_diagnostics` | `diagnostic_msgs/DiagnosticArray` | System diagnostics    |

### Subscribed Topics
| Topic Name | Message Type         | Description                     |
|------------|----------------------|---------------------------------|
| `/rtcm`    | `mavros_msgs/RTCM`   | RTCM messages from base station |
| `/nmea`    | `nmea_msgs/Sentence` | NMEA messages from rover        |

### Monitor Topics
```bash
# Monitor baseline status
ros2 topic echo /baseline_status

# Monitor GNSS status
ros2 topic echo /gnss_status

# Monitor diagnostics
ros2 topic echo /rtk_diagnostics
```

## Custom Messages
This package uses custom message types defined in the [rtk_interfaces](https://github.com/xsenssupport/rtk_interfaces) package. Please ensure you have this package installed.

### BaselineStatus
```yaml
std_msgs/Header header
float64 distance_3d
float64 distance_2d
float64 height_difference
sensor_msgs/NavSatFix base_position
sensor_msgs/NavSatFix rover_position
uint8 satellite_count
float32 average_snr
string rtk_status
```

### GNSSStatus
```yaml
std_msgs/Header header
uint8 satellite_count
float32[] snr_values
uint16[] rtcm_msg_ids
string[] satellite_systems
string coordinate_system
```

### Satellite
```yaml
std_msgs/Header header
string satellite_id
float32 carrier_phase
float32 pseudorange
float32 doppler
float32 snr
uint8 lock_time_indicator
bool half_cycle_valid
```

## Configuration
The node can be configured through ROS2 parameters or launch file arguments. See the launch file for available options.

## Troubleshooting
Common issues and solutions:

1. **No RTCM messages received**
   - Check your RTCM source connection
   - Verify the `/rtcm` topic is being published
   - Check message rates and baud rates

2. **No baseline calculation**
   - Ensure both base position and rover position are being received
   - Check NMEA message format and parsing
