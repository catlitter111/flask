# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 Humble-based self-following robot system that combines computer vision, human detection, and robot control. The system uses stereo vision, ByteTracker for multi-object tracking, WebSocket communication, and DLRobot hardware drivers.

## Key Components

### Core Packages
- **following_robot**: Main package containing vision processing, human detection, tracking, and control nodes
- **dlrobot_robot_python**: Python-based robot hardware driver (alternative to C++ version)
- **custom_msgs**: Custom ROS2 message definitions for the system
- **dlrobot_robot_msg**: Hardware-specific message definitions

### Main Nodes
- **bytetracker_node**: Multi-object tracking using ByteTracker algorithm
- **feature_extraction_node**: Human feature extraction service
- **websocket_bridge_node**: WebSocket communication bridge
- **robot_control_node**: Robot motion control and following logic
- **dlrobot_robot_node**: Hardware driver for DLRobot chassis
- **stereo_vision_node**: Stereo camera processing

## Build and Development Commands

### Environment Setup
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Install dependencies
./install_dependencies.sh

# Set up debugging environment
source debug_setup.sh
```

### Building the System
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select following_robot

# Build with custom messages first
colcon build --packages-select custom_msgs dlrobot_robot_msg
colcon build --packages-select following_robot dlrobot_robot_python

# Source the built packages
source install/setup.bash
```

### Running the System

#### Complete System Launch
```bash
# Launch full system (basic configuration)
ros2 launch following_robot full_system.launch.py

# Launch with custom WebSocket server
ros2 launch following_robot full_system.launch.py websocket_host:=192.168.1.100

# Launch with Ackermann steering
ros2 launch following_robot full_system.launch.py use_ackermann:=true wheelbase:=0.143
```

#### Individual Component Testing
```bash
# Test ByteTracker node
ros2 run following_robot bytetracker_node

# Test stereo vision
ros2 run following_robot stereo_vision_node

# Test WebSocket bridge
ros2 run following_robot websocket_bridge_node

# Test robot control
ros2 run following_robot robot_control_node

# Test hardware driver
ros2 run dlrobot_robot_python dlrobot_robot_node
```

#### Test Scripts
```bash
# Camera testing
python3 test_camera.py

# Full system integration test
python3 test_full_system_launch.py

# WebSocket integration test
python3 test_websocket_robot_integration.py

# Feature extraction test
python3 test_feature_data_flow.py
```

## System Architecture

### Data Flow
1. **Camera Input** → Stereo Vision Node → Image Processing
2. **Image Processing** → ByteTracker Node → Object Detection & Tracking
3. **Tracking Results** → Robot Control Node → Motion Commands
4. **Motion Commands** → DLRobot Driver → Hardware Control
5. **All Data** → WebSocket Bridge → External Communication

### Key Topics
- `/camera/image_raw`: Raw camera feed
- `/bytetracker/tracked_persons`: Detected and tracked persons
- `/cmd_vel`: Robot velocity commands
- `/ackermann_cmd`: Ackermann steering commands (if enabled)
- `/odom`: Robot odometry data
- `/mobile_base/sensors/imu_data`: IMU sensor data

### Services
- `/features/extract_features`: Feature extraction service
- `/stereo/get_distance`: Distance measurement service

## Configuration

### Hardware Configuration
- **DLRobot Models**: Support for mini_akm (0.143m), senior_akm (0.320m), top_akm_bs (0.503m), top_akm_dl (0.549m)
- **Serial Communication**: Default `/dev/ttyS7` at 115200 baud
- **Camera**: Default camera index 0, supports stereo vision

### Software Configuration
- **WebSocket Server**: Default `101.201.150.96:1234`
- **Tracking Mode**: `single` or `multi` person tracking
- **Image Quality**: 1-100 (default 80)
- **Frame Rate**: Default 30 FPS

## Development Notes

### Model Files
- `data/best3.rknn`: Main detection model
- `data/yolov8_pose.rknn`: Pose detection model
- `data/demo_person_features.xlsx`: Target person features

### Dependencies
- ROS2 Humble
- OpenCV (cv2)
- NumPy
- openpyxl (for Excel file handling)
- scipy, lap (for ByteTracker)
- pyserial (for hardware communication)
- websockets (for WebSocket communication)

### Testing
- All test scripts are in the root directory with `test_*.py` naming
- Use `colcon test --packages-select <package_name>` for unit tests
- Debug mode available via `debug_setup.sh`

## Hardware Integration

### Serial Port Setup
- Default serial port: `/dev/ttyS7` (configurable)
- Baud rate: 115200
- May require udev rules setup (see `dlrobot_udev.sh`)

### Camera Setup
- Supports USB cameras (default /dev/video0)
- Stereo vision capabilities
- Configurable resolution and frame rate

### Robot Control Modes
- **Differential Drive**: Standard mode using cmd_vel
- **Ackermann Steering**: Advanced steering with cmd_vel_to_ackermann conversion

## Common Issues and Solutions

### Build Issues
- Ensure custom message packages are built first
- Check ROS2 environment is properly sourced
- Verify all dependencies are installed

### Runtime Issues
- Check serial port permissions and availability
- Verify camera device access
- Ensure WebSocket server is reachable
- Check model files are present in data/ directory

### Performance Optimization
- Adjust image quality and frame rate for bandwidth
- Configure tracking mode (single vs multi) based on use case
- Tune robot control parameters for specific hardware