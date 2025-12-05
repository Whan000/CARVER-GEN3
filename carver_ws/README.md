# CARVER-GEN3 Autonomous Robot Platform

## Complete Technical Documentation

**Version:** 3.0  
**ROS2 Distribution:** Humble  
**Operating System:** Ubuntu 22.04 LTS  
**License:** MIT  

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [System Architecture](#2-system-architecture)
3. [Hardware Requirements](#3-hardware-requirements)
4. [Software Dependencies](#4-software-dependencies)
5. [Installation Guide](#5-installation-guide)
6. [Package Descriptions](#6-package-descriptions)
7. [Hardware Setup](#7-hardware-setup)
8. [Configuration](#8-configuration)
9. [Usage Instructions](#9-usage-instructions)
10. [Launch Files Reference](#10-launch-files-reference)
11. [Control System](#11-control-system)
12. [Localization and Mapping](#12-localization-and-mapping)
13. [Troubleshooting](#13-troubleshooting)
14. [Advanced Topics](#14-advanced-topics)
15. [Development Guidelines](#15-development-guidelines)
16. [References](#16-references)

---

## 1. Project Overview

### 1.1 Introduction

CARVER-GEN3 is an advanced autonomous mobile robot platform featuring four-wheel independent steering, real-time LiDAR-based localization, and multiple path tracking control algorithms. The system is built on ROS2 Humble and designed for research and development in autonomous navigation, mobile robotics, and control systems.

**GitHub Repository:** https://github.com/Whan000/CARVER-GEN3

**Key Technologies:**
- ROS2 Humble on Ubuntu 22.04
- MOLA SLAM for mapping
- NDT-OMP for real-time localization
- Stanley and Pure Pursuit path tracking controllers
- ODrive v3.6 motor controllers with velocity control
- Micro-ROS for real-time embedded control
- Livox MID360 3D LiDAR

### 1.2 Key Features

**Mechanical Design**
- Four-wheel configuration with rear-wheel drive
- Front wheels for steering (Ackermann-like steering geometry)
- Two rear wheels powered by ODrive controllers
- Compact and maneuverable platform

**Sensing Capabilities**
- Livox MID360 3D LiDAR for environment perception and SLAM
- BNO055 9-DOF IMU for orientation and motion sensing
- AMT212EV absolute encoders for precise front wheel steering angle feedback
- Optional Intel RealSense cameras for visual odometry

**Actuation System**
- ODrive v3.6 motor controllers (2 units for rear drive wheels)
- Independent control of left and right rear drive motors
- Front steering actuators for Ackermann steering
- Real-time velocity and position control

**Control Architecture**
- Multiple path tracking algorithms: Stanley, Pure Pursuit
- Ackermann steering kinematics with rear-wheel drive
- Automatic steering follows preplanned paths
- Manual and autonomous operation modes
- Safety monitoring and emergency stop

**Localization and Mapping**
- MOLA (Modular Optimization framework for Localization and Mapping) for SLAM
- NDT-OMP for point cloud registration
- PCL-based localization
- Map-based autonomous navigation

### 1.3 Technical Specifications

**Computing Requirements (Minimum)**
- CPU: Quad-core x86-64 @ 2.0GHz+
- RAM: 16GB DDR4
- Storage: 256GB SSD
- OS: Ubuntu 22.04 LTS

**Computing Requirements (Recommended)**
- CPU: 8+ cores, 3.0GHz+ (e.g., Intel i7/i9, AMD Ryzen 7/9)
- RAM: 24-32GB DDR4/DDR5
- Storage: 512GB NVMe SSD
- GPU: Optional NVIDIA GPU for acceleration

**Tested Configuration:**
- ASUS ROG G17 (2025): Ryzen 9 7945HX, RTX 4060, 24GB DDR5
- Provides excellent performance with headroom for development

**Microcontroller Specifications**
- MCU: STM32G474RE (ARM Cortex-M4F @ 170MHz) x3 units
- Flash: 512KB per unit
- RAM: 128KB per unit
- Real-time capable with hardware FPU
- USB Full-Speed for micro-ROS communication

**Mechanical Specifications**
- Configuration: 4-wheel with rear-wheel drive
- Wheelbase: 0.8m (configurable in URDF)
- Steering: Front Ackermann steering
- Maximum speed: 1-2.5 m/s (configurable, safety-limited)
- Steering range: ±0.6 rad (±34°)

**Sensor Performance**
- LiDAR: Livox MID360 - 360° horizontal FOV, up to 70m range, 200K points/sec
- IMU: BNO055 - 100Hz update rate with onboard sensor fusion
- Encoders: AMT212EV - 12-bit absolute position (4096 positions/rev)
- Localization: 10-20 Hz update rate with NDT-OMP

**Control Performance**
- Control loop: 50 Hz (20ms cycle time)
- Localization update: 10 Hz
- Path tracking accuracy: <0.1m cross-track error (when tuned)
- Response time: <100ms from command to actuation

---

## 2. System Architecture

### 2.1 Overall Architecture

The CARVER system follows a hierarchical architecture with five main layers:

**Layer 1: Hardware Interface Layer**
- ODrive motor controllers (2 units for rear drive wheels)
- Micro-ROS firmware on STM32 microcontrollers
- Sensor drivers (LiDAR, IMU, encoders)
- Front steering actuator interface

**Layer 2: Robot Abstraction Layer**
- URDF robot description
- Ackermann steering kinematics
- Joint state publishers

**Layer 3: Control Layer**
- Path tracking controllers (Stanley, Pure Pursuit)
- Ackermann steering control
- Mode management
- Safety monitoring

**Layer 4: Perception Layer**
- LiDAR processing
- MOLA SLAM for mapping
- Localization (NDT-OMP)

**Layer 5: Application Layer**
- Mission planning
- User interfaces
- Data logging and analysis

### 2.2 Component Diagram

```
                        [Application Layer]
                                |
                    [Mission Planning / UI]
                                |
        +------------------------+------------------------+
        |                                                 |
[Perception Layer]                              [Control Layer]
        |                                                 |
    [MOLA SLAM]                                  [Path Controllers]
    [NDT-OMP]                                    [Mode Manager]
    [PCL Localization]                           [Safety Monitor]
        |                                                 |
        +------------------------+------------------------+
                                |
                    [Robot Abstraction Layer]
                                |
                [Ackermann Kinematics / TF / URDF]
                                |
        +------------------------+------------------------+
        |                        |                        |
[Livox LiDAR]            [Micro-ROS]              [ODrive Interface]
        |                        |                        |
   [MID360]              [STM32 Firmware]         [2x ODrive - Rear]
                                |
                    +-----------+-----------+
                    |           |           |
                [BNO055]   [Encoders]   [Steering]
```

### 2.3 ROS2 Node Architecture

**Sensor Nodes**
- `/livox_lidar_publisher`: Publishes point cloud data
- `/bno055_imu`: IMU data converter and publisher
- `/micro_ros_agent` (multiple instances): Bridge between STM32 firmware and ROS2

**State Estimation Nodes**
- `/robot_state_publisher`: Publishes robot TF tree
- `/localization_node`: Estimates robot pose from LiDAR
- `/fast_lio`: SLAM node for mapping

**Control Nodes**
- `/carver_mode_node`: Manages operation modes
- `/carver_manual_steering`: Manual control interface
- `/carver_stanley` / `/carver_purepursuit` / `/carver_mpc`: Path tracking controllers
- `/carver_kinematic`: Inverse kinematics solver

**Actuation Nodes**
- `/carver_odrive_node`: ODrive motor controller interface

### 2.4 Data Flow

**Sensor Data Flow**
```
LiDAR → /livox/lidar → Fast-LIO → /Odometry
                              ↓
                          Localization → /current_pose
                              ↓
IMU → Micro-ROS → /imu/data → Controller
                              ↓
Encoders → Micro-ROS → /encoder_*/data → Kinematics
```

**Control Data Flow**
```
Reference Path → Controller → /cmd_vel → Kinematics → Wheel Commands → ODrive → Motors
```

### 2.5 Coordinate Frames

**Primary Frames**
- `map`: Fixed world frame
- `odom`: Odometry frame (drift-free locally)
- `base_link`: Robot center frame
- `base_footprint`: Projection of base_link on ground
- `lidar_link`: LiDAR sensor frame
- `imu_link`: IMU sensor frame
- `left_wheel_link`: Left wheel frame
- `right_wheel_link`: Right wheel frame

**Transform Tree**
```
map
 └─ odom
     └─ base_link
         ├─ lidar_link
         ├─ imu_link
         ├─ left_wheel_link
         └─ right_wheel_link
```

---

## 3. Hardware Requirements

### 3.1 Computing Hardware

**Main Computer Requirements:**

**Minimum Specifications:**
- CPU: Quad-core x86-64 processor (Intel Core i5/i7 or AMD Ryzen 5/7 or higher)
- RAM: 16 GB DDR4 (24 GB+ recommended for heavy SLAM)
- GPU: Not required for basic operation (optional for acceleration)
- OS: Ubuntu 22.04 LTS (required)
- Storage: 256 GB SSD minimum (512 GB+ recommended)
- Connectivity: Multiple USB 3.0 ports, Ethernet port

**Recommended Specifications:**
- CPU: 8+ cores, 16+ threads for optimal performance
- RAM: 24-32 GB DDR5
- GPU: NVIDIA GPU (optional, for accelerated point cloud processing)
- Storage: NVMe SSD for fast data logging

**Tested Configuration (Known Working):**
- **Model:** ASUS ROG G17 (2025 Model)
- **CPU:** AMD Ryzen 9 7945HX (16 cores, 32 threads, up to 5.4GHz)
- **GPU:** NVIDIA GeForce RTX 4060 (8GB GDDR6)
- **RAM:** 24 GB DDR5
- **OS:** Ubuntu 22.04 LTS
- **Notes:** High-performance laptop provides excellent performance for real-time control, SLAM, and point cloud processing. GPU useful for future vision features.

**Alternative Platforms (Should Work):**
- High-end gaming laptops (ASUS, MSI, Lenovo Legion, etc.)
- Desktop workstations with sufficient CPU/RAM
- Intel NUC or similar mini-PCs (with adequate specs)
- NVIDIA Jetson AGX Orin (for embedded deployment)

**Why These Specs:**
- Multi-core CPU: Parallel processing for ROS2 nodes, SLAM, and localization
- 16+ GB RAM: Point cloud processing and map storage
- SSD: Fast read/write for data logging and map files
- USB 3.0: High-speed communication with ODrive and sensors
- Ubuntu 22.04: ROS2 Humble native support

**Microcontrollers: STM32G474RE**
- Platform: STM32G474RE (3 units)
  - ARM Cortex-M4F @ 170MHz
  - 512KB Flash, 128KB RAM
  - FPU for floating-point operations
  - USB 2.0 Full-Speed
  - Multiple SPI, I2C, UART interfaces
  
- Unit 1: Main interface and encoder reading (AMT212EV x4)
- Unit 2: Steering control
- Unit 3: BNO055 IMU interface

**Why STM32G474RE:**
- High performance Cortex-M4F core at 170MHz
- Sufficient memory for micro-ROS stack
- Hardware FPU for sensor calculations
- Multiple communication peripherals
- USB connectivity for micro-ROS agent
- Good real-time performance for control applications

### 3.2 Sensors

**Primary Sensors**
- Livox MID360 3D LiDAR
  - Field of view: 360° horizontal
  - Range: Up to 70m
  - Connection: Ethernet or USB
  - Power: 12V DC

- BNO055 9-DOF IMU
  - Sensor fusion built-in
  - Connection: I2C to STM32
  - Power: 3.3V or 5V

- AMT212EV Absolute Encoders (4 units)
  - Resolution: 12-bit (4096 positions per revolution)
  - Interface: SPI
  - Power: 5V DC

**Optional Sensors**
- Intel RealSense L515 or T265
- Additional IMU for redundancy

### 3.3 Actuators

**Motor Controllers**
- ODrive v3.6 (2 units)
  - Each controls one rear drive wheel motor
  - Dual-axis capability (M0 and M1) per ODrive
  - Typically using M0 (Axis 0) for each rear wheel
  - Connection: USB 2.0 to main computer
  - Power input: 24V DC
  - Maximum current: Configurable per motor

**Rear Drive Motors**
- Left rear wheel motor: Brushless DC with Hall sensors
- Right rear wheel motor: Brushless DC with Hall sensors
- Motor encoders: Built-in Hall sensors for commutation
- Power transmission: Direct drive or geared

**Front Steering System**
- Front left steering actuator (servo or stepper motor)
- Front right steering actuator (servo or stepper motor)
- Ackermann steering linkage geometry
- AMT212EV encoders for steering angle feedback
- Controlled via STM32 or separate controller

**Steering Mechanism:**
- Ackermann steering geometry (car-like)
- Front wheels independently steered
- Path tracking via steering angle + rear wheel velocities
- Typical steering range: ±30-40 degrees

### 3.4 Power System

**Power Distribution**
- Main battery: 24V LiPo or Li-ion (6S or higher)
  - Capacity: 5000 mAh minimum recommended
  - Discharge rating: Sufficient for motor current draw

- DC-DC Converter 24V to 12V (5A+)
  - For computer and LiDAR
  
- DC-DC Converter 12V to 5V (3A+)
  - For microcontrollers and encoders

**Safety Components**
- Main power switch
- Emergency stop button (normally closed)
- Fuse or circuit breaker for main battery line
- Individual fuses for each ODrive

### 3.5 Mechanical Platform

**Chassis and Structure**
- Four-wheel independent steering mechanism
- Mounting plates for electronics
- Sensor mounting brackets
- Cable management system
- Protective enclosures for sensitive electronics

**Steering Mechanism**
- Parallel steering linkages or independent servo steering
- Mechanical limits for steering angles
- Encoder mounting on steering axes

---

## 4. Software Dependencies

### 4.1 Core ROS2 Packages

**ROS2 Base**
```bash
sudo apt install ros-humble-desktop-full
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-xacro
```

**Controllers and Gazebo**
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
```

**Navigation and Transforms**
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
sudo apt install ros-humble-tf2-tools
sudo apt install ros-humble-tf-transformations
```

### 4.2 Perception Libraries

**Point Cloud Library (PCL)**
```bash
sudo apt install libpcl-dev
sudo apt install ros-humble-pcl-ros
sudo apt install ros-humble-pcl-conversions
sudo apt install pcl-tools
```

**LiDAR Drivers**
```bash
# Livox SDK2 and ROS2 driver
# Install from source - see Livox documentation
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make
sudo make install

# Livox ROS2 driver
git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
```

**SLAM Packages**
```bash
# MOLA (Modular Optimization framework for Localization and Mapping)
# Install from source - see MOLA documentation
git clone https://github.com/MOLAorg/mola.git src/mola
# Follow MOLA build instructions

# Additional dependencies for MOLA
sudo apt install libgoogle-glog-dev
sudo apt install libgflags-dev
sudo apt install libyaml-cpp-dev
```

### 4.3 Micro-ROS

**Micro-ROS Agent**
```bash
sudo apt install ros-humble-micro-ros-agent

# Or build from source for latest features
git clone https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

**Micro-ROS Firmware Dependencies**
```bash
# For STM32G474RE development
sudo apt install gcc-arm-none-eabi
sudo apt install stm32flash
sudo apt install openocd

# STM32CubeIDE (optional, for firmware development)
# Download from STMicroelectronics website
```

**STM32G474RE Programming:**
```bash
# Using OpenOCD
openocd -f interface/stlink.cfg -f target/stm32g4x.cfg

# Using STM32CubeProgrammer (recommended)
# GUI tool from STMicroelectronics
```

### 4.4 Python Dependencies

**Core Python Packages**
```bash
pip3 install numpy scipy matplotlib
pip3 install transforms3d
pip3 install pyserial
pip3 install pyyaml
```

**ODrive**
```bash
pip3 install odrive
# Or for specific version
pip3 install odrive==0.6.7
```

**Control and Optimization**
```bash
pip3 install control
pip3 install cvxpy  # For MPC controller
pip3 install casadi  # Alternative for MPC
```

### 4.5 Build Tools

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
sudo apt install python3-vcstool
```

---

## 5. Installation Guide

### 5.1 System Preparation

**Required System:**
- Computer meeting minimum specifications (see Section 3.1)
- Ubuntu 22.04 LTS installed
- Internet connection for package downloads
- USB ports for ODrive and STM32 connections

**Step 1: Verify Ubuntu 22.04 Installation**

```bash
lsb_release -a
# Should show: Ubuntu 22.04.x LTS

# Check system resources
lscpu | grep "Model name"
free -h  # Should show 16GB+ for optimal performance
df -h    # Check storage space
```

**Step 2: Update System**
```bash
sudo apt update && sudo apt upgrade -y
```

**Step 3: Install NVIDIA Drivers (Optional, if you have NVIDIA GPU)**

For systems with NVIDIA GPU (e.g., RTX 4060 or similar):
```bash
# Check if driver installed
nvidia-smi

# If not installed or needs update:
sudo apt install nvidia-driver-535  # Or latest recommended
# Or use Additional Drivers tool: Settings > Software & Updates > Additional Drivers

# Reboot after installation
sudo reboot

# Verify after reboot
nvidia-smi
```

**Note:** GPU acceleration is optional. The system works fine with CPU-only processing. GPU can be useful for:
- Accelerated point cloud processing
- Computer vision tasks
- Neural network inference (future features)

**Tested Configuration:**
This system has been tested and confirmed working on:
- ASUS ROG G17 (2025) with Ryzen 9 7945HX, RTX 4060, 24GB RAM
- Similar high-performance laptops should work equivalently

**Step 2: Install ROS2 Humble**
```bash
# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop -y
```

**Step 3: Configure Environment**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Step 4: Initialize rosdep**
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### 5.2 Workspace Setup

**Step 1: Create Workspace**
```bash
mkdir -p ~/carver_ws/src
cd ~/carver_ws
```

**Step 2: Clone Repository**
```bash
cd ~/carver_ws/src
git clone https://github.com/Whan000/CARVER-GEN3.git carver
```

**Step 3: Clone Additional Dependencies**
```bash
# Livox ROS2 driver
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# MOLA (Modular Optimization framework for Localization and Mapping)
# Follow MOLA installation instructions from:
# https://github.com/MOLAorg/mola

# Additional packages as needed
```

**Step 4: Install Dependencies**
```bash
cd ~/carver_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Step 5: Install Python Dependencies**
```bash
pip3 install -r src/carver/requirements.txt
# Or install manually as listed in section 4.4
```

### 5.3 Build Workspace

**Build All Packages**
```bash
cd ~/carver_ws
colcon build --symlink-install
```

**Build Specific Package**
```bash
colcon build --packages-select carver_description
```

**Build with Debug Information**
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

**Source Workspace**
```bash
source ~/carver_ws/install/setup.bash
echo "source ~/carver_ws/install/setup.bash" >> ~/.bashrc
```

### 5.4 Verify Installation

**Check Installed Packages**
```bash
ros2 pkg list | grep carver
```

Expected output:
```
amt212ev_interfaces
bno055_imu
carver_bringup
carver_controller
carver_description
carver_kinematic
carver_manager
carver_odrive
carver_simulation
lidar_localization_ros2
ndt_omp_ros2
```

**Test Package**
```bash
ros2 launch carver_description simple_display.launch.py
```

This should open RViz with the robot model displayed.

### 5.5 ODrive Configuration

**Step 1: Install ODrive Tool**
```bash
pip3 install odrive
```

**Step 2: Connect ODrive**
```bash
# Check USB connection
lsusb | grep ODrive

# Launch ODrive tool
odrivetool
```

**Step 3: Configure ODrive**
```bash
cd ~/carver_ws/odrive_config

# Review configuration
cat OdriveConfig.json

# Upload configuration
python3 uploadConfig.py

# Or follow instructions in uploadConfig.txt
```

**Step 4: Test ODrive**
```python
# In odrivetool:
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# Wait for calibration
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_pos = 1
# Motor should move
```

### 5.6 Micro-ROS Firmware Setup

**Step 1: Navigate to Firmware Directory**
```bash
cd ~/carver_ws/firmware
```

**Step 2: Flash AMT212E Encoder Firmware**
```bash
cd AMT212E-V_Micro-ROS
# Follow README instructions for flashing
# Typically involves:
# 1. Connect STM32 via USB
# 2. Build firmware
# 3. Flash using openocd or STM32CubeProgrammer
```

**Step 3: Flash BNO055 IMU Firmware**
```bash
cd ../bno055_uros_publisher
# Follow README instructions for flashing
```

**Step 4: Flash Carver Interface Firmware**
```bash
cd ../Carver_Interface
# Follow README instructions for flashing
```

**Step 5: Test Micro-ROS Connection**
```bash
# Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 2000000

# In another terminal, check topics
ros2 topic list
# Should see topics from micro-ROS nodes
```

### 5.7 Device Permissions

**Grant USB Device Access**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Create udev rules for ODrive
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' | sudo tee /etc/udev/rules.d/91-odrive.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Logout and login for group changes to take effect
```

### 5.8 Network Configuration (for Livox LiDAR)

**Configure Static IP**
```bash
# If using Ethernet connection to LiDAR
# Edit netplan configuration
sudo nano /etc/netplan/01-netcfg.yaml
```

Add:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:  # Replace with your interface name
      addresses:
        - 192.168.1.50/24  # Your computer IP
      routes:
        - to: 192.168.1.0/24
          via: 192.168.1.1
```

Apply:
```bash
sudo netplan apply
```

Test connection:
```bash
ping 192.168.1.10  # Default Livox IP
```

---

## 6. Package Descriptions

### 6.1 Core Packages

#### carver_description

**Purpose:** URDF robot description and visualization configuration

**Contents:**
- `urdf/`: XACRO files defining robot structure
  - `carver_main.xacro`: Main robot definition
  - `carver_core.xacro`: Core structural elements
  - `carver_controller.xacro`: Controller configurations
  - `carver_inertia.xacro`: Inertial properties
  - `carver_material.xacro`: Visual materials and colors
  - `carver_params.xacro`: Robot parameters

- `meshes/`: STL files for visual and collision geometry
  - Contains all robot component meshes

- `launch/`: Launch files
  - `carver.launch.py`: Launches robot state publisher
  - `simple_display.launch.py`: Visualization in RViz

- `config/`: Configuration files
  - `carver_controller_param.yaml`: Controller parameters

- `rviz/`: RViz configuration files
  - `display.rviz`: Default visualization setup

**Key Parameters:**
Located in `carver_params.xacro`:
- Wheelbase dimensions
- Track width
- Wheel radius
- Steering limits
- Mass and inertia properties

**Usage:**
```bash
# View robot in RViz
ros2 launch carver_description simple_display.launch.py

# Include in other launch files
from launch.actions import IncludeLaunchDescription
carver_desc = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('carver_description'),
        '/launch/carver.launch.py'
    ])
)
```

#### carver_bringup

**Purpose:** System startup and integration launch files

**Contents:**
- `launch/`: Launch files for different operation modes
  - `bringup.launch.py`: Main system startup
  - `manual_steering.launch.py`: Manual control mode
  - `slam.launch.py`: SLAM and mapping mode
  - `uros.launch.py`: Micro-ROS agents only

**Launch File Details:**

**bringup.launch.py**
Starts complete robot system:
- Micro-ROS agents (3 instances)
- BNO055 IMU converter
- Robot state publisher
- ODrive motor controller interface
- Mode manager
- Manual steering interface
- Static TF publishers (map→odom→base_link)

**manual_steering.launch.py**
Minimal system for manual control:
- Micro-ROS agent for interface
- Manual steering node
- ODrive motor controller

**slam.launch.py**
System for mapping:
- Micro-ROS agent for BNO055
- Livox LiDAR driver
- BNO055 converter
- Robot description
- Manual steering
- MOLA SLAM (if configured)

**uros.launch.py**
Three micro-ROS agents:
- Interface agent: /dev/ttyACM1
- Steering agent: /dev/ttyACM2
- BNO055 agent: /dev/ttyACM0

**Usage:**
```bash
# Full system startup
ros2 launch carver_bringup bringup.launch.py

# Manual control only
ros2 launch carver_bringup manual_steering.launch.py

# SLAM mode
ros2 launch carver_bringup slam.launch.py
```

#### carver_controller

**Purpose:** Path tracking and trajectory following algorithms

**Contents:**
- `scripts/`: Python controller implementations
  - `carver_stanley.py`: Stanley controller
  - `carver_purepursuit.py`: Pure Pursuit controller
  - `carver_mpc.py`: Model Predictive Control
  - `carver_combined.py`: Combined adaptive controller

- `path/`: Pre-defined trajectory files
  - `trajectory.yaml`: Default trajectory
  - `trajectory750.yaml`: Trajectory with 750mm parameters
  - `trajectory1000.yaml`: Trajectory with 1000mm parameters

**Controller Descriptions:**

**Stanley Controller**
- Lateral control based on cross-track error and heading error
- Suitable for high-speed operation
- Parameters: k_e (cross-track gain), k_heading (heading gain), k_v (velocity gain)

**Pure Pursuit Controller**
- Geometric path following using look-ahead distance
- Simple and robust
- Parameters: lookahead_distance, min_lookahead, max_lookahead

**MPC Controller**
- Optimal control over prediction horizon
- Handles constraints
- Parameters: prediction_horizon, control_horizon, Q_matrix, R_matrix

**Combined Controller**
- Adaptive switching between controllers
- Condition-based selection
- Automatic tuning

**Usage:**
```bash
# Run Stanley controller
ros2 run carver_controller carver_stanley.py

# Run with parameters
ros2 run carver_controller carver_stanley.py --ros-args -p k_e:=0.5 -p k_heading:=1.0
```

#### carver_kinematic

**Purpose:** Differential drive kinematic models and transformations

**Contents:**
- `scripts/`:
  - `carver_ik.py`: Inverse kinematics for differential drive
  - `carver_converter.py`: Velocity and coordinate conversions

**Functions:**
- Forward kinematics: wheel velocities → robot velocity
- Inverse kinematics: desired velocity/steering → wheel velocities
- Coordinate transformations
- Velocity profile generation

**Usage:**
```bash
ros2 run carver_kinematic carver_ik.py
```

The node subscribes to `/cmd_vel` (or `/steering_angle` and `/target_speed`) and publishes individual wheel velocity commands for the ODrive controllers.

#### carver_odrive

**Purpose:** ODrive motor controller interface

**Contents:**
- `scripts/`:
  - `carver_odrive.py`: Main ODrive interface node
  - `dummy_steering.py`: Testing and simulation

**Features:**
- Real-time motor control
- Position, velocity, and torque modes
- Safety monitoring
- Error handling and recovery
- Watchdog timers

**Parameters:**
- `control_mode`: 'position', 'velocity', or 'torque'
- `max_velocity`: Maximum wheel velocity
- `max_current`: Current limit per motor

**Usage:**
```bash
ros2 run carver_odrive carver_odrive.py --ros-args -p control_mode:=velocity
```

#### carver_manager

**Purpose:** Robot mode management and manual control

**Contents:**
- `scripts/`:
  - `carver_mode.py`: Mode management node
  - `carver_manual_steering.py`: Manual control interface

**Operation Modes:**
- **Manual Mode:** Direct user control via keyboard/joystick
- **Autonomous Mode:** Controller-based path following
- **Emergency Stop:** Immediate halt of all motion

**Manual Control Keys:**
- W/S: Forward/backward
- A/D: Steering left/right
- Q/E: Rotate in place
- Space: Emergency stop
- ESC: Exit manual mode

**Usage:**
```bash
# Start mode manager
ros2 run carver_manager carver_mode.py

# Start manual steering
ros2 run carver_manager carver_manual_steering.py
```

#### carver_simulation

**Purpose:** Gazebo simulation environment

**Contents:**
- `launch/`:
  - `simulation-full.launch.py`: Complete simulation
  - `simple_robot.launch.py`: Minimal simulation

- `worlds/`:
  - `custom_world.sdf`: Custom environment
  - `custom_world_classic.world`: Classic format world

- `scripts/`:
  - `simulation_teleop.py`: Teleoperation in simulation
  - `odom_tf_relay.py`: TF relay for simulation

**Features:**
- Physics-based dynamics
- Sensor simulation (LiDAR, IMU)
- Environment modeling
- Real-time visualization

**Usage:**
```bash
# Launch full simulation
ros2 launch carver_simulation simulation-full.launch.py

# Teleoperate simulated robot
ros2 run carver_simulation simulation_teleop.py
```

### 6.2 Sensor Packages

#### bno055_imu

**Purpose:** BNO055 IMU sensor interface and data conversion

**Contents:**
- `scripts/`:
  - `bno055_imu.py`: Main converter node

**Features:**
- Converts raw micro-ROS IMU messages to standard ROS2 format
- Publishes orientation, angular velocity, linear acceleration
- Provides calibration status

**Published Topics:**
- `/imu/data` (sensor_msgs/Imu): Processed IMU data
- `/imu/calibration_status`: Calibration status

**Usage:**
```bash
ros2 run bno055_imu bno055_imu.py
```

#### amt212ev_interfaces

**Purpose:** Custom message definitions for AMT212EV encoders

**Contents:**
- `msg/`:
  - `AmtRead.msg`: Encoder reading message

**Message Definition:**
```
# AMT212EV encoder reading
float32 angle       # Absolute angle in radians
uint16 position     # Raw 12-bit position value
bool error          # Error flag
```

### 6.3 Localization Packages

#### lidar_localization_ros2

**Purpose:** LiDAR-based localization using point cloud matching

**Contents:**
- `include/pcl_localization/`:
  - `pcl_localization_component.hpp`: Main localization component
  - `lidar_undistortion.hpp`: Motion compensation

- `src/`:
  - `pcl_localization_component.cpp`: Implementation
  - `pcl_localization_node.cpp`: Node wrapper

- `launch/`:
  - `pcl_localization.launch.py`: Launch file

- `param/`:
  - `localization.yaml`: Configuration parameters

**Features:**
- Real-time point cloud registration
- NDT-based scan matching
- Map loading and management
- Pose estimation and publishing

**Parameters:**
In `localization.yaml`:
```yaml
localization_frequency: 10.0
ndt_resolution: 1.0
transformation_epsilon: 0.01
maximum_iterations: 35
voxel_leaf_size: 0.1
```

**Usage:**
```bash
ros2 launch lidar_localization_ros2 pcl_localization.launch.py map_path:=/path/to/map.pcd
```

#### ndt_omp_ros2

**Purpose:** OpenMP-accelerated NDT implementation

**Contents:**
- `include/pclomp/`:
  - `ndt_omp.h`: NDT algorithm header
  - `gicp_omp.h`: GICP alternative
  - Implementation headers

- `src/pclomp/`:
  - `ndt_omp.cpp`: NDT implementation
  - `gicp_omp.cpp`: GICP implementation

**Features:**
- Multi-threaded NDT computation
- Significantly faster than standard PCL NDT
- GICP as alternative registration method

**Usage:**
Used as a library by `lidar_localization_ros2` package.

---

## 7. Hardware Setup

### 7.1 Power System Assembly

**Step 1: Prepare Power Distribution**

Install in order:
1. Main battery connector
2. Master power switch
3. Fuse or circuit breaker (rated for total current)
4. Power distribution board

**Step 2: Connect ODrives**

For each ODrive (2 total):
```
Battery (+) → [Fuse 30A] → ODrive VCC
Battery (-) → ODrive GND
```

Recommended fuse values:
- Per ODrive: 30A fast-blow
- Main battery line: 40A slow-blow (for 2 ODrives)

**Step 3: Install DC-DC Converters**

**24V to 12V Converter:**
```
Input:  Battery 24V (+/-) → Converter IN
Output: Converter OUT → Computer 12V
                      → LiDAR 12V
                      → 12V-5V converter IN
```

**12V to 5V Converter:**
```
Input:  12V rail → Converter IN
Output: Converter OUT → STM32 5V (all 3 units)
                      → Encoders 5V (wheel encoders if used)
                      → IMU 5V
```

**Step 4: Emergency Stop Integration**

Wire emergency stop button:
```
Battery (+) → E-Stop NC → Main Switch → System
```

When pressed, E-Stop opens circuit, cutting all power.

### 7.2 ODrive Motor Connections

**Configuration for Rear-Wheel Drive:**

Two ODrive units control the left and right rear drive wheels. Each ODrive's Axis 0 (M0) is used for one rear wheel motor.

**Wiring Pattern:**

**ODrive 1 - Left Rear Drive Wheel:**
```
M0 (Axis 0): Left Rear Wheel Motor
    Phase A → A
    Phase B → B
    Phase C → C

Hall Sensors:
    Hall A → Hall A
    Hall B → Hall B
    Hall C → Hall C
    Hall 5V → 5V
    Hall GND → GND

Power:
    VCC → 24V Battery (+) through fuse
    GND → Battery (-)

USB:
    USB → Computer USB Port
```

**ODrive 2 - Right Rear Drive Wheel:**
```
M0 (Axis 0): Right Rear Wheel Motor
    Phase A → A
    Phase B → B
    Phase C → C

Hall Sensors:
    Hall A → Hall A
    Hall B → Hall B
    Hall C → Hall C
    Hall 5V → 5V
    Hall GND → GND

Power:
    VCC → 24V Battery (+) through fuse
    GND → Battery (-)

USB:
    USB → Computer USB Port
```

**Note:** Each ODrive's M1 (Axis 1) is available for expansion or can be used for additional functions. The front steering actuators are controlled separately (typically via STM32 or dedicated servo controller).

### 7.3 Micro-ROS / STM32G474RE Connections

**Three STM32G474RE Units Configuration:**

**STM32G474RE Key Specifications:**
- Core: ARM Cortex-M4F @ 170MHz
- Flash: 512KB
- RAM: 128KB
- USB: Full-Speed Device
- ADC: 12-bit, up to 5 MSPS
- Timers: Advanced control, general purpose
- Package: LQFP64

**Unit 1: Main Interface and Encoders**
Device: /dev/carver_interface (ttyACM1)

```
STM32G474RE Connections:
Power:
- 5V  → VDD pins
- GND → VSS pins

SPI1 - Front Left Encoder (AMT212EV):
  PA5  → SCK  (SPI1_SCK)
  PA6  → MISO (SPI1_MISO)
  PA4  → CS   (SPI1_NSS)

SPI2 - Front Right Encoder (AMT212EV):
  PB13 → SCK  (SPI2_SCK)
  PB14 → MISO (SPI2_MISO)
  PB12 → CS   (SPI2_NSS)

SPI3 - Rear Left Encoder (AMT212EV):
  PC10 → SCK  (SPI3_SCK)
  PC11 → MISO (SPI3_MISO)
  PA15 → CS   (SPI3_NSS)

GPIO - Rear Right Encoder (AMT212EV):
  PB3  → SCK
  PB4  → MISO
  PB5  → CS

USB:
  PA11 → USB_DM (D-)
  PA12 → USB_DP (D+)
  
UART (Debug, optional):
  PA2  → USART2_TX
  PA3  → USART2_RX
```

**Unit 2: Steering Control**
Device: /dev/carver_steering (ttyACM2)

```
STM32G474RE Connections:
Power:
- 5V, GND from power supply

GPIO for Steering Control:
  PC0-PC7  → Steering control signals
  PB0-PB1  → PWM outputs (if needed)

USB:
  PA11 → USB_DM (D-)
  PA12 → USB_DP (D+)
```

**Unit 3: BNO055 IMU**
Device: /dev/carver_bno055 (ttyACM0)

```
STM32G474RE Connections:
Power:
- 5V, GND from power supply

I2C1 - BNO055:
  PB8  → SCL (I2C1_SCL) with 4.7kΩ pull-up to 3.3V
  PB9  → SDA (I2C1_SDA) with 4.7kΩ pull-up to 3.3V

Alternative I2C3:
  PA8  → SCL (I2C3_SCL)
  PC9  → SDA (I2C3_SDA)

USB:
  PA11 → USB_DM (D-)
  PA12 → USB_DP (D+)

GPIO (Optional):
  PC13 → BNO055 Reset
  PC14 → BNO055 Interrupt
```

**Important STM32G474RE Notes:**

1. **Power Supply:**
   - VDD voltage: 2.0V to 3.6V (typically 3.3V from regulator)
   - Digital I/O: 5V tolerant with proper configuration
   - USB requires 3.3V supply

2. **Clock Configuration:**
   - External crystal: 8MHz or 24MHz (if used)
   - PLL to 170MHz system clock
   - USB clock derived from PLL

3. **USB Configuration:**
   - Full-Speed (12 Mbps)
   - Requires USB pull-up on D+ line
   - Micro-ROS uses USB CDC (Virtual COM Port)

4. **SPI Configuration for AMT212EV:**
   - Mode: 0 (CPOL=0, CPHA=0)
   - Clock speed: ≤ 2 MHz
   - 16-bit transfers
   - MSB first

5. **I2C Configuration for BNO055:**
   - Standard mode (100 kHz) or Fast mode (400 kHz)
   - 7-bit addressing
   - Pull-up resistors required: 4.7kΩ typical
   - BNO055 address: 0x28 or 0x29 (depends on COM3 pin)

### 7.4 BNO055 IMU Wiring

**Connection to STM32:**
```
BNO055 Breakout → STM32
VCC (5V)        → 5V
GND             → GND
SDA             → PB9 (I2C1_SDA)
SCL             → PB8 (I2C1_SCL)
RST (optional)  → GPIO for reset
INT (optional)  → GPIO for interrupt
```

**Important Notes:**
- Use 4.7kΩ pull-up resistors on SDA and SCL
- Some BNO055 breakouts include pull-ups
- Check breakout board specifications for voltage requirements (3.3V vs 5V)

### 7.5 AMT212EV Encoder Setup

**For 2-Wheel Drive Configuration:**

The AMT212EV absolute encoders can be used for:
- Left wheel position feedback
- Right wheel position feedback
- Steering position (if applicable)
- Additional auxiliary sensing

**Mechanical Mounting:**
1. Mount encoder coaxially with wheel shaft or steering axis
2. Ensure no wobble or play
3. Use flexible coupling if needed
4. Align zero position with mechanical reference

**Electrical Connection (per encoder):**
```
AMT212EV → STM32G474RE
VCC (5V) → 5V
GND      → GND
CS       → Unique GPIO per encoder
SCK      → SPI SCK (shared)
MISO     → SPI MISO (shared)
MOSI     → Leave unconnected (read-only)
```

**SPI Configuration:**
- Mode: 0 (CPOL=0, CPHA=0)
- Clock speed: ≤ 2 MHz
- Bit order: MSB first

**Typical Usage:**
- 2 encoders for wheel odometry (left and right wheels)
- Additional encoders for steering position feedback
- Up to 4 encoders can be interfaced with STM32 unit 1

### 7.6 Livox MID360 LiDAR Setup

**Physical Installation:**
1. Mount at chest height (~0.5-1.0m above ground)
2. Ensure 360° unobstructed view
3. Level within ±5°
4. Vibration isolate from motors

**Electrical Connection:**

**Option A: Ethernet Connection**
```
LiDAR Power:
12V DC → LiDAR power input
GND → Common ground

Data:
Ethernet cable → Computer Ethernet port
Configure static IP on computer (see section 5.8)
LiDAR default IP: 192.168.1.10
```

**Option B: USB Connection**
```
LiDAR Power: 12V DC
Data: USB cable → Computer
```

**Network Configuration:**
```bash
# Set computer IP
sudo ifconfig eth0 192.168.1.50 netmask 255.255.255.0

# Test connection
ping 192.168.1.10

# Access LiDAR web interface
# Open browser: http://192.168.1.10
```

### 7.7 Cable Management

**Best Practices:**

1. **Separate Power and Signal:**
   - Route power cables away from signal cables
   - Use shielded cables for encoders and IMU
   - Keep ODrive motor wires twisted

2. **Strain Relief:**
   - Add stress relief at all connectors
   - Use cable ties with cushioning
   - Leave service loops at joints

3. **Labeling:**
   - Label all cables at both ends
   - Use color coding:
     - Red: +24V
     - Black: GND
     - Yellow: +12V
     - Orange: +5V
     - White/Blue: Data

4. **Protection:**
   - Use cable wrap or conduit
   - Protect cables from abrasion
   - Keep away from moving parts and heat sources

### 7.8 Safety System

**Emergency Stop Implementation:**

**Hardware E-Stop:**
```
Battery → E-Stop Button (NC) → Main Switch → System
```

**Software E-Stop:**
```
Monitor in code:
- Watchdog timers
- Communication timeouts
- Sensor failures
- Current overload
- Battery voltage

Action on E-Stop:
1. Set all motor velocities to zero
2. Disable ODrive axes
3. Set emergency flag
4. Log event
```

**E-Stop Recovery:**
```
Manual reset required:
1. Clear error flags
2. Re-enable ODrives
3. Reset controllers
4. Resume operation
```

---

## 8. Configuration

### 8.1 Robot Parameters

**Location:** `carver_description/urdf/carver_params.xacro`

**Key Parameters:**
```xml
<!-- Physical dimensions -->
<xacro:property name="wheelbase_length" value="0.5"/>  <!-- meters -->
<xacro:property name="track_width" value="0.4"/>       <!-- meters -->
<xacro:property name="wheel_radius" value="0.075"/>    <!-- meters -->

<!-- Steering limits -->
<xacro:property name="max_steering_angle" value="0.785"/>  <!-- radians, ~45° -->

<!-- Mass properties -->
<xacro:property name="base_mass" value="25.0"/>  <!-- kg -->
<xacro:property name="wheel_mass" value="1.5"/>  <!-- kg -->

<!-- Inertia (calculated or measured) -->
<!-- See carver_inertia.xacro for detailed inertia matrices -->
```

**Modification Procedure:**
1. Measure physical robot dimensions
2. Update parameters in `carver_params.xacro`
3. Rebuild workspace:
   ```bash
   cd ~/carver_ws
   colcon build --packages-select carver_description
   source install/setup.bash
   ```
4. Verify in RViz:
   ```bash
   ros2 launch carver_description simple_display.launch.py
   ```

### 8.2 ODrive Configuration

**Configuration File:** `odrive_config/OdriveConfig.json`

**Motor Configuration (Axis 0):**

**Motor Type and Physical Parameters:**
```json
"axis0.config.motor.motor_type": 0,          // High current BLDC
"axis0.config.motor.pole_pairs": 4,          // 4 pole pairs
"axis0.config.motor.phase_resistance": 0.0354,  // Measured
"axis0.config.motor.phase_inductance": 0.0000625,  // Measured
"axis0.config.motor.torque_constant": 0.48,  // Nm/A
"axis0.config.motor.direction": 1.0          // Motor direction
```

**Current Limits:**
```json
"axis0.config.motor.current_soft_max": 50.0,    // Soft limit (A)
"axis0.config.motor.current_hard_max": 100.0,   // Hard limit (A)
"axis0.config.motor.calibration_current": 25.0, // Calibration current (A)
"axis0.config.motor.current_control_bandwidth": 5000.0  // Hz
```

**Encoder Configuration (Hall Sensors):**
```json
"hall_encoder0.config.enabled": true,
"hall_encoder0.config.edges_calibrated": true,
"hall_encoder0.config.hall_polarity_calibrated": true,
"hall_encoder0.config.hall_polarity": 0,
"hall_encoder0.config.ignore_illegal_hall_state": false
```

Hall sensor edge calibration values (axis 0):
```json
"hall_encoder0.config.edge0": 0.5972,
"hall_encoder0.config.edge1": 0.7208,
"hall_encoder0.config.edge2": 0.9467,
"hall_encoder0.config.edge3": 0.0562,
"hall_encoder0.config.edge4": 0.2848,
"hall_encoder0.config.edge5": 0.3776
```

**Controller Configuration:**
```json
"axis0.controller.config.control_mode": 2,  // Velocity control
"axis0.controller.config.input_mode": 1,    // Passthrough mode
"axis0.controller.config.pos_gain": 10.0,
"axis0.controller.config.vel_gain": 3.0,
"axis0.controller.config.vel_integrator_gain": 1.633,
"axis0.controller.config.vel_limit": 60.0,  // Rev/s
"axis0.controller.config.vel_ramp_rate": 30.0,  // Rev/s²
"axis0.controller.config.torque_ramp_rate": 40.0  // Nm/s
```

**Velocity and Torque Limits:**
```json
"axis0.controller.config.vel_limit": 60.0,            // Max velocity (rev/s)
"axis0.controller.config.vel_limit_tolerance": 1.3125,
"axis0.controller.config.enable_vel_limit": true,
"axis0.config.torque_soft_max": Infinity,             // No software torque limit
"axis0.controller.config.enable_torque_mode_vel_limit": true
```

**Commutation Mapper (for Hall sensors):**
```json
"axis0.commutation_mapper.config.circular": true,
"axis0.commutation_mapper.config.circular_output_range": 1.0,
"axis0.commutation_mapper.config.offset": 0.0,
"axis0.commutation_mapper.config.offset_valid": true,
"axis0.commutation_mapper.config.scale": 1.0
```

**Position/Velocity Mapper:**
```json
"axis0.pos_vel_mapper.config.scale": 0.25,  // Gear ratio or scaling
"axis0.pos_vel_mapper.config.circular": false,
"axis0.pos_vel_mapper.config.offset": 0.0
```

**Trapezoidal Trajectory:**
```json
"axis0.trap_traj.config.vel_limit": 50.0,   // Rev/s
"axis0.trap_traj.config.accel_limit": 25.0, // Rev/s²
"axis0.trap_traj.config.decel_limit": 25.0  // Rev/s²
```

**Global Configuration:**

**Power Limits:**
```json
"config.dc_bus_overvoltage_trip_level": 56.0,   // Max voltage (V)
"config.dc_bus_undervoltage_trip_level": 42.0,  // Min voltage (V)
"config.dc_max_positive_current": 50.0,         // Max current (A)
"config.dc_max_negative_current": -50.0,        // Max regen current (A)
"config.max_regen_current": -10.0                // Regen limit (A)
```

**Communication:**
```json
"config.enable_uart_a": true,
"config.uart_a_baudrate": 19200,
"config.uart0_protocol": 3,      // ASCII protocol
"config.usb_cdc_protocol": 3     // ASCII protocol
```

**CAN Configuration (Axis 0):**
```json
"axis0.config.can.node_id": 0,
"axis0.config.can.heartbeat_msg_rate_ms": 100,
"axis0.config.can.encoder_msg_rate_ms": 10
```

**Upload Configuration:**

**Method 1: Using Python Script**
```bash
cd ~/carver_ws/odrive_config
python3 uploadConfig.py
```

**Method 2: Using odrivetool**
```python
# Connect to ODrive
odrivetool

# Load configuration from JSON
import json
with open('OdriveConfig.json', 'r') as f:
    config = json.load(f)

# Apply configuration
for key, value in config.items():
    # Parse nested keys (e.g., "axis0.config.motor.pole_pairs")
    parts = key.split('.')
    obj = odrv0
    for part in parts[:-1]:
        obj = getattr(obj, part)
    setattr(obj, parts[-1], value)

# Save configuration
odrv0.save_configuration()
odrv0.reboot()
```

**Calibration Procedure:**

After uploading configuration, calibrate each axis:

```python
# In odrivetool:
# 1. Full calibration sequence
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

# Wait for completion (~10 seconds)
# Check for errors
odrv0.axis0.error  # Should be 0

# 2. Test closed-loop control
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# 3. Test velocity control
odrv0.axis0.controller.input_vel = 5.0  # 5 rev/s
# Motor should spin smoothly

# 4. Save configuration if calibration successful
odrv0.save_configuration()
odrv0.reboot()
```

**Key Configuration Notes:**

1. **Hall Sensor Commutation:**
   - Uses Hall sensors instead of incremental encoders
   - Hall edges pre-calibrated and stored in configuration
   - Suitable for motors with built-in Hall sensors

2. **Velocity Control Mode:**
   - Primary control mode for wheel motors
   - PID gains tuned for smooth velocity tracking
   - Input filter bandwidth set to 2.0 Hz

3. **Current Limits:**
   - Soft max: 50A for normal operation
   - Hard max: 100A for brief overloads
   - Calibration current: 25A

4. **Voltage Limits:**
   - Designed for 48V (6S) operation
   - Undervoltage trip: 42V
   - Overvoltage trip: 56V

5. **Per-ODrive Configuration:**
   - Each of 4 ODrives needs individual configuration
   - Axis 0 typically controls steering motor
   - Axis 1 typically controls drive motor
   - Hall sensor calibration values unique to each motor

**Verification After Configuration:**

```bash
# Test each ODrive
odrivetool

# For each axis, verify:
odrv0.axis0.config.motor.pole_pairs  # Should be 4
odrv0.axis0.config.motor.phase_resistance  # Should be ~0.035
odrv0.hall_encoder0.config.enabled  # Should be True
odrv0.axis0.controller.config.control_mode  # Should be 2 (velocity)

# Test motor response
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel = 2.0  # Should spin at 2 rev/s
odrv0.axis0.controller.input_vel = 0.0  # Should stop smoothly
```

**Troubleshooting Configuration Issues:**

**Error: MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE**
```python
# Measure resistance again
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
# Wait for completion
# Check measured values
odrv0.axis0.config.motor.phase_resistance  # Should be 0.03-0.1 Ω
```

**Error: ENCODER_ERROR_ILLEGAL_HALL_STATE**
```python
# Check Hall sensor connections
# Verify all 3 Hall wires connected
# May need to recalibrate Hall edges
odrv0.axis0.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
```

**Motor vibrates but doesn't spin:**
```python
# Check Hall sensor edge calibration
odrv0.hall_encoder0.config.edges_calibrated  # Should be True
# If False, recalibrate:
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

### 8.3 Controller Parameters

**Location:** `carver_description/config/carver_controller_param.yaml`

**Stanley Controller:**
```yaml
stanley_controller:
  ros__parameters:
    k_e: 0.5              # Cross-track error gain
    k_heading: 1.0        # Heading error gain
    k_v: 1.0              # Velocity gain
    max_steering_angle: 0.785  # Maximum steering in radians
    wheelbase: 0.5        # Wheelbase length in meters
```

**Pure Pursuit Controller:**
```yaml
purepursuit_controller:
  ros__parameters:
    lookahead_distance: 1.0    # Base look-ahead distance
    min_lookahead: 0.5         # Minimum look-ahead
    max_lookahead: 2.0         # Maximum look-ahead
    lookahead_gain: 0.5        # Speed-dependent gain
    max_steering_angle: 0.785
```

**MPC Controller:**
```yaml
mpc_controller:
  ros__parameters:
    prediction_horizon: 10     # Steps
    control_horizon: 5         # Steps
    dt: 0.1                    # Time step (seconds)
    Q_matrix: [10.0, 10.0, 5.0, 1.0]  # State cost [x, y, theta, v]
    R_matrix: [1.0, 1.0]       # Control cost [v, delta]
    max_velocity: 2.0          # m/s
    max_steering_angle: 0.785  # radians
```

**Loading Parameters:**
```bash
ros2 run carver_controller carver_stanley.py --ros-args --params-file $(ros2 pkg prefix carver_description)/share/carver_description/config/carver_controller_param.yaml
```

### 8.4 Localization Parameters

**Location:** `lidar_localization_ros2/param/localization.yaml`

```yaml
pcl_localization:
  ros__parameters:
    # Update rate
    localization_frequency: 10.0  # Hz
    
    # NDT parameters
    ndt_resolution: 1.0           # Voxel size in meters
    transformation_epsilon: 0.01  # Convergence threshold
    maximum_iterations: 35        # Max optimization iterations
    step_size: 0.1                # Optimization step size
    
    # Preprocessing
    voxel_leaf_size: 0.1          # Downsampling voxel size
    
    # Map handling
    map_update_distance: 5.0      # Meters to trigger map section load
    map_radius: 50.0              # Radius of local map
    
    # Pose estimation
    initial_pose_required: true
    pose_covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
```

**Tuning Guidelines:**

- **ndt_resolution:** 
  - Larger = faster, less accurate
  - Smaller = slower, more accurate
  - Typical: 1.0-2.0m for outdoor, 0.5-1.0m for indoor

- **transformation_epsilon:**
  - Smaller = more precise convergence
  - Larger = faster but less accurate
  - Typical: 0.001-0.01

- **maximum_iterations:**
  - More = better convergence, slower
  - Fewer = faster, may not converge
  - Typical: 30-50

### 8.5 Sensor Calibration

**IMU Calibration:**

The BNO055 has built-in calibration. Monitor status:
```bash
ros2 topic echo /imu/calibration_status
```

Status values (0-3, where 3 is fully calibrated):
- `sys`: System calibration
- `gyro`: Gyroscope calibration
- `accel`: Accelerometer calibration
- `mag`: Magnetometer calibration

**Calibration Procedure:**
1. Place robot on level surface
2. Slowly rotate around all axes
3. Hold various orientations for 2-3 seconds each
4. Continue until all values reach 3

**Encoder Calibration:**

**Zero Position Calibration:**
1. Manually align all steering to straight-ahead
2. Record encoder readings
3. Set as zero offset in firmware or software

**Verification:**
```bash
ros2 topic echo /encoder_fl/data
ros2 topic echo /encoder_fr/data
ros2 topic echo /encoder_rl/data
ros2 topic echo /encoder_rr/data
```

All should read ~0 when steering is straight.

**LiDAR Calibration:**

**Extrinsic Calibration (LiDAR to Robot Frame):**
Measure and set in URDF:
```xml
<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.2 0.0 0.3" rpy="0 0 0"/>
  <!-- Adjust xyz and rpy to match physical mounting -->
</joint>
```

**Verification:**
```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Should match physical measurements.

---

## 9. Usage Instructions

### 9.1 System Startup

**Complete System Startup Sequence:**

**Step 1: Power On**
```bash
# 1. Ensure emergency stop is released
# 2. Turn on main power switch
# 3. Verify all power LEDs illuminate
# 4. Check voltage levels with multimeter if first power-on
```

**Step 2: Start Core System**
```bash
# Open terminal 1
ros2 launch carver_bringup bringup.launch.py
```

This starts:
- Micro-ROS agents (3 instances)
- IMU converter
- Robot state publisher
- ODrive interface
- Mode manager
- Manual steering
- Static transforms

**Step 3: Verify System Status**
```bash
# Open terminal 2
ros2 node list
```

Expected nodes:
```
/bno055_imu
/carver_interface_agent
/carver_steering_agent
/carver_bno055_agent
/carver_odrive_node
/carver_mode_node
/carver_manual_steering
/robot_state_publisher
/static_map_to_odom
/static_odom_to_base_link
```

**Step 4: Check Topics**
```bash
ros2 topic list
```

Critical topics to verify:
```
/imu/data
/encoder_fl/data
/encoder_fr/data
/encoder_rl/data
/encoder_rr/data
/livox/lidar
/cmd_vel
/joint_states
/tf
/tf_static
```

**Step 5: Monitor Data**
```bash
# Check IMU
ros2 topic echo /imu/data --once

# Check encoders
ros2 topic echo /encoder_fl/data --once

# Check LiDAR
ros2 topic echo /livox/lidar --once
```

**Step 6: Launch Visualization**
```bash
# Terminal 3
rviz2 -d $(ros2 pkg prefix carver_description)/share/carver_description/rviz/display.rviz
```

### 9.2 Manual Control Mode

**Start Manual Control:**

If not already running from bringup:
```bash
ros2 run carver_manager carver_manual_steering.py
```

**Control Keys:**

**Movement:**
- `W`: Forward
- `S`: Backward
- `A`: Steer left
- `D`: Steer right
- `Q`: Rotate counter-clockwise
- `E`: Rotate clockwise

**Speed Control:**
- `R`: Increase speed
- `F`: Decrease speed

**Safety:**
- `Space`: Emergency stop
- `ESC`: Exit manual mode

**Tips:**
- Start with low speed until familiar with controls
- Test in open area first
- Keep emergency stop accessible
- Monitor battery voltage

### 9.3 SLAM and Mapping

**Start SLAM Mode:**
```bash
ros2 launch carver_bringup slam.launch.py
```

This starts:
- Livox LiDAR driver
- BNO055 IMU
- Manual steering (for driving during mapping)
- MOLA SLAM (if configured)

**Drive to Map:**
```bash
# In separate terminal
ros2 run carver_manager carver_manual_steering.py
```

**Mapping Best Practices:**
1. Drive slowly (0.3-0.5 m/s)
2. Make smooth turns
3. Overlap coverage for loop closure
4. Avoid rapid acceleration
5. Cover entire area systematically

**Monitor Mapping:**
```bash
# Open RViz
rviz2

# Add displays:
# - PointCloud2: /livox/lidar
# - PointCloud2: /mola/map (accumulated map)
# - Path: /mola/trajectory
# - PoseStamped: /state_estimator/pose
```

**Save Map:**

After mapping complete:
```bash
# Save map (if MOLA configured for auto-save)
# Map typically saved to: ~/carver_ws/maps/

# Or use service if available:
ros2 service call /mola/save_map std_srvs/srv/Trigger

# Verify saved map
pcl_viewer ~/carver_ws/maps/your_map.pcd
```

**Map Storage Location:**
```bash
# Default location (check your configuration)
~/carver_ws/maps/
```

### 9.4 Autonomous Navigation with Preplanned Paths

The CARVER controllers follow preplanned trajectories loaded from YAML files. This approach is ideal for repetitive tasks, testing, racing, or known environments.

**Prerequisites:**
1. Map created and localization system working
2. Trajectory file created (YAML format)
3. Controllers tuned for your robot
4. Safety systems verified

**Step 1: Create Trajectory File**

Create a YAML file with your desired path:

**Location:** `~/carver_ws/src/carver_controller/path/my_trajectory.yaml`

**Format Example:**
```yaml
waypoints:
  - x: 0.0
    y: 0.0
    yaw: 0.0
  - x: 5.0
    y: 0.0
    yaw: 0.0
  - x: 10.0
    y: 2.0
    yaw: 0.3
  - x: 15.0
    y: 5.0
    yaw: 0.5
  - x: 20.0
    y: 5.0
    yaw: 0.0
```

**Trajectory Planning Tips:**
- Include yaw (heading) for each waypoint when using Stanley controller
- Space waypoints 0.5-2.0m apart for smooth tracking
- Ensure path is feasible (no sharp turns beyond steering limits)
- Test in simulation first
- Add safety margin from obstacles

**Step 2: Start System**
```bash
# Terminal 1: Main system
ros2 launch carver_bringup bringup.launch.py

# Terminal 2: Localization
ros2 launch lidar_localization_ros2 pcl_localization.launch.py map_path:=/path/to/your/map.pcd
```

**Step 3: Set Initial Pose**

**Using RViz:**
1. Open RViz with localization visualization
2. Click "2D Pose Estimate" button
3. Click and drag on map to set robot's current pose

**Using Command:**
```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --once
```

**Step 4: Verify Localization**

Check that localization has converged:
```bash
# Monitor pose
ros2 topic echo /state_estimator/pose

# Should show stable position matching robot's actual location
```

**Step 5: Start Path Tracking Controller**

**Stanley Controller (Recommended):**
```bash
ros2 run carver_controller carver_stanley.py --ros-args \
  -p path_file:=/home/katana/Desktop/array/carver_ws/src/carver_controller/path/my_trajectory.yaml \
  -p target_speed:=1.25 \
  -p k_heading:=0.25 \
  -p k_crosstrack:=0.1
```

**Pure Pursuit Controller:**
```bash
ros2 run carver_controller carver_purepursuit.py --ros-args \
  -p waypoint_file:=/home/katana/Desktop/array/carver_ws/src/carver_controller/path/my_trajectory.yaml \
  -p target_speed:=1.0 \
  -p min_lookahead:=3.0 \
  -p max_lookahead:=5.0
```

**Step 6: Enable Controller**

Controllers start in disabled state for safety. Enable when ready:
```bash
ros2 service call /auto/enable std_srvs/srv/SetBool "data: true"
```

The robot will now follow the trajectory!

**Step 7: Monitor Execution**

**In Terminal:**
```bash
# Watch controller output
# Stanley shows: waypoint index, CTE, heading error, steering angle
# Pure Pursuit shows: lookahead distance, target point, steering angle

# Monitor actual commands
ros2 topic echo /steering_angle
ros2 topic echo /target_speed
```

**In RViz:**
- Reference path visualization (green)
- Current target point (red sphere)
- Robot pose (from localization)
- LiDAR scan

**Step 8: Emergency Stop**

**Hardware E-Stop:**
- Press emergency stop button on robot

**Software Disable:**
```bash
ros2 service call /auto/enable std_srvs/srv/SetBool "data: false"
```

**Manual Override:**
```bash
# Stop commands
ros2 topic pub /target_speed std_msgs/msg/Float32 "data: 0.0" --once
ros2 topic pub /steering_angle std_msgs/msg/Float32 "data: 0.0" --once
```

### 9.5 Creating Optimal Trajectories

**Method 1: Manual Waypoint Definition**

Drive robot manually and record positions:
```bash
# Start recording poses
ros2 topic echo /state_estimator/pose > recorded_path.txt

# Drive robot along desired path using manual control

# Stop recording (Ctrl+C)

# Parse text file to create YAML
python3 parse_trajectory.py recorded_path.txt output.yaml
```

**Method 2: Path Planning in RViz**

Use RViz plugins to define waypoints visually:
```bash
# Install interactive markers if not already installed
sudo apt install ros-humble-interactive-markers

# Use RViz to click waypoints
# Export to YAML format
```

**Method 3: Programmatic Generation**

Generate smooth trajectories programmatically:
```python
import numpy as np
import yaml

def generate_circle_trajectory(radius, num_points):
    """Generate circular trajectory"""
    waypoints = []
    for i in range(num_points):
        theta = 2 * np.pi * i / num_points
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        yaw = theta + np.pi/2  # Tangent direction
        waypoints.append({'x': float(x), 'y': float(y), 'yaw': float(yaw)})
    
    return {'waypoints': waypoints}

# Generate and save
trajectory = generate_circle_trajectory(radius=10.0, num_points=50)
with open('circle_trajectory.yaml', 'w') as f:
    yaml.dump(trajectory, f)
```

**Method 4: Optimization-Based**

Use trajectory optimization for time-optimal or energy-optimal paths:
```python
from scipy.optimize import minimize
import numpy as np

def trajectory_cost(waypoints, obstacles, time_weight=1.0, energy_weight=1.0):
    """Cost function for trajectory optimization"""
    # Calculate total path length
    path_length = np.sum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1))
    
    # Calculate curvature (steering effort)
    curvature = calculate_path_curvature(waypoints)
    
    # Obstacle penalties
    obstacle_cost = calculate_obstacle_distance_cost(waypoints, obstacles)
    
    total_cost = time_weight * path_length + energy_weight * curvature + obstacle_cost
    return total_cost

# Optimize trajectory
# ... (implementation details)
```

### 9.6 Trajectory Validation

Before running on real robot, validate trajectories:

**Check 1: Path Feasibility**
```python
def check_steering_feasibility(waypoints, max_steering_angle, wheelbase):
    """Check if path curvature is within steering limits"""
    for i in range(len(waypoints) - 2):
        p0, p1, p2 = waypoints[i:i+3]
        
        # Calculate required steering angle
        curvature = calculate_curvature(p0, p1, p2)
        required_steering = np.arctan(wheelbase * curvature)
        
        if abs(required_steering) > max_steering_angle:
            print(f"Warning: Waypoint {i+1} requires {np.degrees(required_steering):.1f}° steering")
            return False
    return True
```

**Check 2: Visualization**
```bash
# Visualize path in RViz
ros2 run carver_controller carver_stanley.py  # Loads and publishes path
# Check in RViz that path looks correct
```

**Check 3: Simulation Test**
```bash
# Test in simulation first
ros2 launch carver_simulation simulation-full.launch.py
# Load trajectory and test controller behavior
```

### 9.7 Multi-Lap Operation

For repetitive tasks or testing, run multiple laps:

**Method 1: Loop Trajectory**

Create closed-loop trajectory (last waypoint near first):
```yaml
waypoints:
  - x: 0.0
    y: 0.0
  # ... intermediate points ...
  - x: 0.5
    y: 0.0  # Near starting point
```

Controller will automatically loop when reaching the end.

**Method 2: Script Multiple Runs**

```bash
#!/bin/bash
# run_multiple_laps.sh

for lap in {1..5}; do
    echo "Starting lap $lap"
    
    # Enable controller
    ros2 service call /auto/enable std_srvs/srv/SetBool "data: true"
    
    # Wait for completion (adjust time based on lap duration)
    sleep 120
    
    # Reset waypoint index (if controller supports)
    ros2 service call /reset_waypoints std_srvs/srv/Empty
    
    echo "Lap $lap complete"
done
```

### 9.8 Performance Logging

Log data for analysis:
```bash
# Record all relevant topics
ros2 bag record \
  /state_estimator/pose \
  /steering_angle \
  /target_speed \
  /imu \
  /livox/lidar \
  -o lap_$(date +%Y%m%d_%H%M%S)
```

**Analyze Lap Performance:**
```python
import rosbag2_py
import matplotlib.pyplot as plt

def analyze_lap_performance(bag_file):
    # Extract pose data
    poses = extract_poses_from_bag(bag_file)
    
    # Calculate metrics
    lap_time = poses[-1].timestamp - poses[0].timestamp
    path_length = calculate_path_length(poses)
    avg_speed = path_length / lap_time
    max_cross_track_error = calculate_max_cte(poses, reference_path)
    
    print(f"Lap time: {lap_time:.2f}s")
    print(f"Average speed: {avg_speed:.2f}m/s")
    print(f"Max cross-track error: {max_cross_track_error:.3f}m")
    
    # Plot trajectory
    plot_trajectory(poses, reference_path)
```

### 9.5 Simulation Testing

**Launch Simulation:**
```bash
ros2 launch carver_simulation simulation-full.launch.py
```

**Control in Simulation:**
```bash
# Terminal 2
ros2 run carver_simulation simulation_teleop.py
```

**Test Controllers in Simulation:**
```bash
# Load test trajectory
ros2 topic pub /reference_trajectory nav_msgs/msg/Path ...

# Start controller
ros2 run carver_controller carver_stanley.py
```

**Advantages of Simulation Testing:**
- Safe environment for testing
- Repeatability
- No hardware damage risk
- Algorithm development and tuning
- System integration verification

---

## 10. Launch Files Reference

### 10.1 bringup.launch.py

**Purpose:** Main system startup

**Components Launched:**
1. `uros.launch.py` - Micro-ROS agents
2. `bno055_imu.py` - IMU converter
3. `carver.launch.py` - Robot description
4. `carver_odrive.py` - Motor controller
5. `carver_mode.py` - Mode manager
6. `carver_manual_steering.py` - Manual control
7. Static TF: map → odom
8. Static TF: odom → base_link

**Parameters:**
- ODrive control_mode: 'velocity' (default)

**Usage:**
```bash
ros2 launch carver_bringup bringup.launch.py
```

**When to Use:**
- Normal robot operation
- Full system startup
- Hardware testing

### 10.2 manual_steering.launch.py

**Purpose:** Minimal system for manual control

**Components Launched:**
1. Micro-ROS agent (interface only)
2. Motor controller
3. Manual steering

**Parameters:**
- Device: /dev/carver_interface
- Baud rate: 2000000
- Control mode: velocity

**Usage:**
```bash
ros2 launch carver_bringup manual_steering.launch.py
```

**When to Use:**
- Manual driving only
- Quick movement tests
- Before full system ready

### 10.3 slam.launch.py

**Purpose:** SLAM and mapping mode using MOLA

**Components Launched:**
1. Micro-ROS agent (BNO055)
2. Livox LiDAR driver
3. BNO055 IMU converter
4. Robot description
5. Manual steering
6. MOLA SLAM (if configured)

**Parameters:**
- Device: /dev/carver_bno055
- Baud rate: 2000000

**Usage:**
```bash
ros2 launch carver_bringup slam.launch.py
```

**When to Use:**
- Creating new maps with MOLA
- Mapping unknown environments
- Updating existing maps

**Note:** MOLA SLAM configuration may need to be enabled in the launch file or configured separately depending on your installation.

### 10.4 uros.launch.py

**Purpose:** Micro-ROS agents only

**Components Launched:**
Three micro-ROS agents:
1. Interface agent: /dev/ttyACM1
2. Steering agent: /dev/ttyACM2
3. BNO055 agent: /dev/ttyACM0

**Parameters:**
- Baud rate: 2000000 (all agents)
- Transport: serial

**Usage:**
```bash
ros2 launch carver_bringup uros.launch.py
```

**When to Use:**
- Micro-ROS testing
- Firmware development
- Sensor debugging

### 10.5 Launch File Customization

**Modify Device Names:**

Edit launch file to match your system:
```python
# In uros.launch.py
arguments=[
    'serial',
    '-b', '2000000',
    '--dev', '/dev/ttyACM0'  # Change to your device
]
```

**Find Device Names:**
```bash
# List all USB serial devices
ls -l /dev/ttyACM*
ls -l /dev/ttyUSB*

# Or use dmesg
dmesg | grep tty

# Create udev rules for persistent names
sudo nano /etc/udev/rules.d/99-carver.rules
```

Example udev rule:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", SYMLINK+="carver_bno055"
```

**Add Custom Parameters:**
```python
motor_node = Node(
    package='carver_odrive',
    executable='carver_odrive.py',
    name='carver_odrive_node',
    output='screen',
    parameters=[
        {'control_mode': 'velocity'},
        {'max_velocity': 5.0},
        {'max_current': 15.0},
    ]
)
```

---

## 11. Control System

### 11.1 Control Architecture

**Hierarchical Control:**

```
Mission Planner (High Level)
        ↓
Path Tracking Controller (Mid Level)
        ↓
Inverse Kinematics (Low Level)
        ↓
Motor Controllers (Actuator Level)
```

### 11.2 Stanley Controller

**Algorithm:**

The Stanley controller computes steering angle based on:
1. Heading error (computed from velocity vector)
2. Cross-track error (perpendicular distance to path)

**Implementation Details:**

**Subscribed Topics:**
- `/state_estimator/pose` (nav_msgs/Odometry): Robot pose from localization
- `/imu` (sensor_msgs/Imu): Orientation data

**Published Topics:**
- `/steering_angle` (std_msgs/Float32): Steering command in radians
- `/target_speed` (std_msgs/Float32): Target velocity in m/s
- `/path_visualization` (nav_msgs/Path): Visualize reference path
- `/target_point` (visualization_msgs/Marker): Current target point

**Services:**
- `/auto/enable` (std_srvs/SetBool): Enable/disable controller

**Steering Command:**
```
δ = k_heading * θ_e + arctan(k_crosstrack * e / (ks + v))

Where:
δ = steering angle
θ_e = heading error (computed from velocity vector to target)
k_heading = heading error gain
k_crosstrack = cross-track error gain
e = cross-track error (perpendicular distance to nearest path segment)
ks = softening constant
v = current velocity
```

**Parameters:**

```yaml
k_heading: 0.25          # Heading error gain
k_crosstrack: 0.1        # Cross-track error gain
ks_gain: 0.1             # Velocity softening constant
target_speed: 1.25       # Target velocity (m/s)
max_steer: 0.6           # Maximum steering angle (rad, ~34°)
steering_sign: 1.0       # Steering direction multiplier
lookahead_distance: 5.0  # Distance to target point (m)
path_file: "/path/to/trajectory.yaml"  # Waypoint file
```

**Tuning Guide:**

**k_heading (Heading Error Gain):**
- Default: 0.25
- Increase: More aggressive heading correction
- Decrease: Gentler turns
- Typical range: 0.1-0.5

**k_crosstrack (Cross-track Error Gain):**
- Default: 0.1
- Increase: Faster lateral error correction
- Decrease: Smoother path following
- Typical range: 0.05-0.3

**ks_gain (Softening Constant):**
- Default: 0.1
- Increase: Less aggressive at low speeds
- Decrease: More responsive at low speeds
- Typical range: 0.05-0.5

**lookahead_distance:**
- Default: 5.0m
- Increase: Smoother but may cut corners
- Decrease: Tighter tracking but more oscillation
- Typical range: 3.0-10.0m

**Key Features:**
- Uses velocity vector for heading calculation (not just IMU yaw)
- Separate calculation of heading error (toward lookahead target) and cross-track error (from nearest segment)
- Monotonic waypoint advancement (never goes backward)
- Speed maintained at target unless disabled

**Advantages:**
- Robust heading estimation from velocity
- Separate heading and lateral error terms
- Works well with preplanned paths
- Smooth path following

**Disadvantages:**
- Requires motion for velocity-based heading
- May overshoot on very sharp turns
- Needs calibration for specific vehicle

**Best For:**
- Preplanned trajectory following
- Medium to high-speed operation
- Outdoor navigation
- Racing or time-trial applications

### 11.3 Pure Pursuit Controller

**Algorithm:**

Computes steering to reach a look-ahead point on the path using robust target selection.

**Implementation Details:**

**Subscribed Topics:**
- `/state_estimator/pose` (nav_msgs/Odometry): Robot pose from localization
- `/imu` (sensor_msgs/Imu): Orientation and angular velocity

**Published Topics:**
- `/steering_angle` (std_msgs/Float32): Steering command in radians
- `/target_speed` (std_msgs/Float32): Target velocity in m/s
- `/path_visualization` (nav_msgs/Path): Visualize reference path
- `/target_point` (visualization_msgs/Marker): Current lookahead target

**Services:**
- `/auto/enable` (std_srvs/SetBool): Enable/disable controller

**Steering Command:**
```
δ = arctan(2 * L * sin(α) / l_d)

Where:
δ = steering angle
L = wheelbase (0.8m default)
α = angle to look-ahead point
l_d = look-ahead distance
```

**Dynamic Look-ahead Distance:**
```
l_d = min_lookahead + lookahead_gain * v
l_d = clip(l_d, min_lookahead, max_lookahead)

Where:
v = current velocity
```

**Parameters:**

```yaml
wheelbase: 0.8               # Distance between axles (m)
max_steering_angle: 0.6      # Maximum steering (rad, ~34°)
min_lookahead: 3.0           # Minimum lookahead distance (m)
max_lookahead: 5.0           # Maximum lookahead distance (m)
lookahead_gain: 1.0          # Speed-dependent lookahead gain
target_speed: 1.0            # Target velocity (m/s)
max_speed: 2.5               # Maximum velocity limit (m/s)
waypoint_file: "/path/to/trajectory.yaml"
```

**Target Point Selection Algorithm:**

The controller uses a robust approach:
1. Find nearest waypoint to robot
2. Ensure monotonic forward movement (no backtracking)
3. Walk forward along path until accumulated distance equals lookahead
4. Interpolate target point within segment
5. Fallback to last waypoint if lookahead extends beyond path

**Speed Adaptation:**

Speed automatically reduces in turns:
```python
speed_factor = 1.0 - abs(steering_angle) / max_steering * 0.5
target_velocity = target_speed * speed_factor
```

**Tuning Guide:**

**min_lookahead / max_lookahead:**
- Defaults: 3.0m / 5.0m
- Increase: Smoother, may cut corners more
- Decrease: Tighter tracking, more oscillation
- Typical range: 1.0-10.0m

**lookahead_gain:**
- Default: 1.0
- Increase: More speed-dependent behavior
- Decrease: More consistent lookahead
- Typical range: 0.5-2.0

**target_speed:**
- Default: 1.0 m/s
- Adjust based on environment and safety
- Automatically reduces in sharp turns
- Typical range: 0.5-2.5 m/s

**Waypoint File Format:**

The controller supports multiple YAML formats:

**Option 1: Simple list**
```yaml
waypoints:
  - x: 0.0
    y: 0.0
  - x: 5.0
    y: 0.0
  - x: 10.0
    y: 2.0
```

**Option 2: Array format**
```yaml
waypoints:
  - [0.0, 0.0]
  - [5.0, 0.0]
  - [10.0, 2.0]
```

**Option 3: Pose format**
```yaml
poses:
  - pose:
      position:
        x: 0.0
        y: 0.0
  - pose:
      position:
        x: 5.0
        y: 0.0
```

**Key Features:**
- Dynamic velocity-dependent lookahead
- Automatic speed reduction in turns
- Monotonic waypoint progression
- Multiple YAML format support
- Robust target point interpolation
- Completion detection (stops near final waypoint)

**Advantages:**
- Simple and intuitive
- Very robust
- Good for smooth paths
- Easy to tune
- Automatic turn speed adjustment

**Disadvantages:**
- Cuts corners on sharp turns
- Less precise than Stanley
- May be sluggish at low speeds

**Best For:**
- Smooth curved paths
- Low to medium speeds
- Initial controller testing
- Indoor navigation
- Scenarios where smoothness is prioritized over precision

### 11.4 Model Predictive Control (MPC)

**Algorithm:**

Solves optimization problem over prediction horizon:

**Cost Function:**
```
J = Σ(||x(k) - x_ref(k)||²_Q + ||u(k)||²_R)

Where:
x = state [x, y, θ, v]
u = control [v_cmd, δ]
Q = state cost matrix
R = control cost matrix
```

**Constraints:**
```
|δ| ≤ δ_max
|v| ≤ v_max
|Δδ| ≤ Δδ_max
|Δv| ≤ Δv_max
```

**Tuning Guide:**

**Prediction Horizon:**
- Default: 10 steps
- Increase: Better long-term behavior, slower computation
- Decrease: Faster computation, more reactive
- Typical range: 5-20 steps

**Q Matrix (State Cost):**
- Higher values: Prioritize tracking accuracy
- Format: [q_x, q_y, q_θ, q_v]
- Example: [10.0, 10.0, 5.0, 1.0]

**R Matrix (Control Cost):**
- Higher values: Smoother control, less aggressive
- Format: [r_v, r_δ]
- Example: [1.0, 1.0]

**Advantages:**
- Optimal control
- Handles constraints explicitly
- Preview capability
- Can optimize multiple objectives

**Disadvantages:**
- Computationally expensive
- Requires tuning
- Complex implementation
- Real-time performance depends on hardware

**Best For:**
- Complex maneuvers
- Constraint satisfaction
- When computational resources available
- Research applications

### 11.5 Combined Controller

**Strategy:**

Adaptively switches between controllers based on:
- Path curvature
- Vehicle speed
- Tracking error
- Environmental conditions

**Switching Logic:**
```python
if high_speed and straight_path:
    use Stanley
elif smooth_curve and low_speed:
    use Pure Pursuit
elif complex_maneuver:
    use MPC
```

**Advantages:**
- Best of all controllers
- Adaptive to conditions
- Robust performance

**Disadvantages:**
- Complex implementation
- Requires tuning of switching conditions
- Potential discontinuities at transitions

**Configuration:**
```yaml
combined_controller:
  ros__parameters:
    # Speed thresholds
    high_speed_threshold: 1.5  # m/s
    low_speed_threshold: 0.5   # m/s
    
    # Curvature thresholds
    high_curvature_threshold: 0.5  # 1/m
    
    # Switching hysteresis
    switching_hysteresis: 0.2
    
    # Default controller
    default_controller: "stanley"
```

### 11.6 Kinematic Model

**Ackermann Steering with Rear-Wheel Drive:**

**State Vector:**
```
x = [x, y, θ, v]

Where:
x, y = position (at rear axle center)
θ = heading
v = velocity
```

**Control Input:**
```
u = [v_rear, δ]

Where:
v_rear = rear wheel velocity (average of left and right)
δ = front steering angle
```

**Kinematic Equations (Bicycle Model):**
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = (v / L) * tan(δ)

Where:
L = wheelbase (distance from front to rear axle)
```

**Ackermann Steering Geometry:**

For proper Ackermann steering, the inner and outer front wheel angles differ:
```
tan(δ_outer) = L / (R + w/2)
tan(δ_inner) = L / (R - w/2)

Where:
R = turning radius
w = track width (distance between left and right wheels)
δ_outer = outer wheel steering angle
δ_inner = inner wheel steering angle
```

**Inverse Kinematics:**

Given desired velocity and steering angle from controller:
```
v_left_rear = v_desired
v_right_rear = v_desired
δ_left_front = calculate_ackermann_left(δ_desired, L, w)
δ_right_front = calculate_ackermann_right(δ_desired, L, w)
```

**For Rear-Wheel Drive:**
```python
def inverse_kinematics(v_desired, steering_angle, wheelbase, track_width):
    """
    Convert desired velocity and steering to wheel commands
    
    Args:
        v_desired: Desired velocity (m/s)
        steering_angle: Desired steering angle (rad)
        wheelbase: Distance front to rear axle (m)
        track_width: Distance between left/right wheels (m)
    
    Returns:
        v_left_rear: Left rear wheel velocity (m/s)
        v_right_rear: Right rear wheel velocity (m/s)
        delta_left: Left front steering angle (rad)
        delta_right: Right front steering angle (rad)
    """
    # Rear wheels same velocity (both driven)
    v_left_rear = v_desired
    v_right_rear = v_desired
    
    # Calculate turning radius
    if abs(steering_angle) < 0.001:
        # Straight line
        delta_left = 0.0
        delta_right = 0.0
    else:
        R = wheelbase / tan(steering_angle)
        
        # Ackermann angles
        delta_inner = atan(wheelbase / (R - track_width/2))
        delta_outer = atan(wheelbase / (R + track_width/2))
        
        if steering_angle > 0:  # Turning left
            delta_left = delta_inner
            delta_right = delta_outer
        else:  # Turning right
            delta_left = delta_outer
            delta_right = delta_inner
    
    return v_left_rear, v_right_rear, delta_left, delta_right
```

**Constraints:**
- Steering angle limits: |δ| ≤ δ_max (typically ±0.6 rad)
- Rear wheel velocity limits: |v| ≤ v_max
- Minimum turning radius: R_min = L / tan(δ_max)
- Maximum velocity in turns (for stability)

**Implementation:**

Located in `carver_kinematic` package.

**Path Tracking with Ackermann Steering:**

The Stanley and Pure Pursuit controllers output desired steering angles which are directly applied to the front wheels, with rear wheels driven at the commanded velocity.

---

## 12. Localization and Mapping

### 12.1 Localization System Overview

The CARVER robot uses LiDAR-based localization for autonomous navigation. The system employs NDT-OMP (Normal Distributions Transform with OpenMP acceleration) for real-time pose estimation against pre-built maps.

**SLAM System:** The workspace includes MOLA (Modular Optimization framework for Localization and Mapping) for creating maps of new environments. Once maps are created, they are used with NDT-OMP for real-time localization during autonomous operation.

### 12.2 MOLA SLAM (Mapping)

**MOLA Overview:**

MOLA is a modular C++ library for SLAM and localization, providing:
- Real-time 3D LiDAR SLAM
- Efficient point cloud processing
- Map optimization
- Various sensor fusion options

**Usage for Mapping:**

```bash
# Start SLAM mode to create new map
ros2 launch carver_bringup slam.launch.py

# Drive robot to map environment
# Use manual control or teleoperation

# Save map when complete
# Map will be saved as PCD file for later use
```

**Map Creation Tips:**
- Drive slowly (0.3-0.5 m/s) for best results
- Ensure good LiDAR coverage of area
- Avoid rapid acceleration or turning
- Close loops when possible for optimization
- Save map in accessible location for later use

### 12.3 NDT-OMP Localization

**Algorithm:**

Normal Distributions Transform (NDT) with OpenMP acceleration for multi-threaded point cloud registration.

**Process:**
1. Load reference map (PCD format) created with MOLA
2. Receive LiDAR scan from `/livox/lidar`
3. Downsample point cloud for efficiency
4. Run NDT matching against reference map
5. Publish estimated pose to `/state_estimator/pose`

**Topics:**

**Subscribed:**
- `/livox/lidar` (sensor_msgs/PointCloud2): Raw point cloud from LiDAR

**Published:**
- `/state_estimator/pose` (nav_msgs/Odometry): Estimated robot pose
- `/localization/status`: Localization quality metrics

**Configuration:**

In `lidar_localization_ros2/param/localization.yaml`:

```yaml
pcl_localization:
  ros__parameters:
    # NDT
    ndt_resolution: 1.0              # Voxel size (m)
    transformation_epsilon: 0.01     # Convergence threshold
    maximum_iterations: 35           # Max optimization iterations
    step_size: 0.1                   # Optimization step size
    
    # Performance
    num_threads: 4                   # OpenMP threads
    
    # Preprocessing
    voxel_leaf_size: 0.1             # Downsampling voxel size (m)
    
    # Map handling
    map_update_distance: 5.0         # Meters to trigger map section load
    map_radius: 50.0                 # Radius of local map (m)
    
    # Pose estimation
    initial_pose_required: true
    localization_frequency: 10.0     # Update rate (Hz)
```

**Tuning for Performance:**

**Faster (less accurate):**
```yaml
ndt_resolution: 2.0
maximum_iterations: 20
voxel_leaf_size: 0.2
localization_frequency: 5.0
```

**More Accurate (slower):**
```yaml
ndt_resolution: 0.5
maximum_iterations: 50
voxel_leaf_size: 0.05
localization_frequency: 20.0
```

**Usage:**
```bash
ros2 launch lidar_localization_ros2 pcl_localization.launch.py map_path:=/path/to/map.pcd
```

### 12.3 Map Management

**Map Format:**

Maps stored as `.pcd` files (Point Cloud Data) for use with NDT-OMP localization.

**Directory Structure:**
```
~/carver_ws/maps/
├── environment1.pcd
├── environment1_metadata.yaml
├── environment2.pcd
└── environment2_metadata.yaml
```

**Metadata File Example:**
```yaml
map_name: "environment1"
creation_date: "2024-12-05"
robot_id: "carver_01"
area: "laboratory"
resolution: 0.1
num_points: 1500000
bounds:
  min_x: -50.0
  max_x: 50.0
  min_y: -50.0
  max_y: 50.0
  min_z: -1.0
  max_z: 3.0
```

**Map Processing:**

**View Map:**
```bash
pcl_viewer ~/carver_ws/maps/environment1.pcd
```

**Convert Format:**
```bash
# PCD to PLY
pcl_pcd2ply ~/carver_ws/maps/environment1.pcd environment1.ply

# Filter map (remove floor/ceiling)
pcl_filter ~/carver_ws/maps/environment1.pcd environment1_filtered.pcd -filter PassThrough -field z -min -1.0 -max 3.0
```

**Downsampling:**
```bash
pcl_voxel_grid ~/carver_ws/maps/environment1.pcd environment1_downsampled.pcd -leaf 0.1,0.1,0.1
```

### 12.4 Initial Pose Setting

For localization to work, an initial pose estimate must be provided:

**Method 1: Using RViz**
1. Open RViz with localization visualization
2. Click "2D Pose Estimate" button in toolbar
3. Click on map at robot's approximate current location
4. Drag to set orientation

**Method 2: Using Topic**
```bash
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
  }
}" --once
```

**Method 3: Saved Positions**

Create a configuration file with known starting positions:
```yaml
# saved_positions.yaml
parking_spot:
  x: 5.0
  y: 2.0
  yaw: 0.0

charging_station:
  x: -3.0
  y: -1.5
  yaw: 1.57
```

Then publish programmatically:
```python
def publish_initial_pose(x, y, yaw):
    # Convert yaw to quaternion
    q = quaternion_from_euler(0, 0, yaw)
    # Publish initial pose
    # ... (implementation)
```

### 12.5 Creating Maps with MOLA

**MOLA (Modular Optimization framework for Localization and Mapping)** is used to create maps for autonomous navigation.

**Prerequisites:**
- MOLA installed in workspace
- Livox LiDAR operational
- IMU calibrated
- Robot able to move (manual control)

**Mapping Procedure:**

**Step 1: Launch SLAM System**
```bash
ros2 launch carver_bringup slam.launch.py
```

This starts:
- Livox MID360 LiDAR driver
- BNO055 IMU
- MOLA SLAM (if configured in launch file)
- Manual control interface

**Step 2: Drive and Map**
```bash
# In another terminal, use manual control
ros2 run carver_manager carver_manual_steering.py

# Or use keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Mapping Best Practices:**
1. **Speed:** Drive slowly and steadily (0.3-0.5 m/s)
2. **Coverage:** Ensure complete area coverage
3. **Overlap:** Drive paths with some overlap for loop closure
4. **Smoothness:** Avoid jerky movements or rapid turns
5. **Features:** Map areas with good geometric features
6. **Loops:** Close loops by returning to starting areas

**Step 3: Monitor Mapping Quality**
```bash
# Watch point cloud registration in RViz
rviz2

# Add these displays:
# - PointCloud2: /livox/lidar (raw scan)
# - PointCloud2: /mola/map (accumulated map)
# - Path: /mola/trajectory (robot path)
# - PoseStamped: /state_estimator/pose (current pose)
```

**Step 4: Save Map**

After completing mapping:
```bash
# MOLA typically saves map automatically to configured location
# Or use service call:
ros2 service call /mola/save_map std_srvs/srv/Trigger

# Map saved as PCD file, for example:
# ~/carver_ws/maps/environment_$(date).pcd
```

**Step 5: Verify Map Quality**
```bash
# View saved map
pcl_viewer ~/carver_ws/maps/your_map.pcd

# Check map statistics
pcl_pcd_info ~/carver_ws/maps/your_map.pcd
```

**MOLA Configuration:**

Typical MOLA parameters (if using custom configuration):
```yaml
mola_slam:
  ros__parameters:
    # Point cloud processing
    voxel_filter_resolution: 0.1  # meters
    
    # Registration
    icp_max_iterations: 50
    icp_max_correspondence_distance: 1.0
    
    # Loop closure
    enable_loop_closure: true
    loop_closure_min_distance: 10.0  # meters
    
    # Map saving
    auto_save_map: true
    map_save_directory: "~/carver_ws/maps"
```

### 12.6 Localization Quality Monitoring

**Indicators of Good Localization:**
1. Stable pose estimates (low jitter)
2. Small position covariance values
3. Consistent scan matching scores
4. Smooth odometry output

**Indicators of Poor Localization:**
1. Large jumps in pose estimates
2. High covariance values
3. Failed scan matching (iterations maxed out)
4. Erratic odometry

**Monitoring Commands:**
```bash
# Check pose estimates
ros2 topic echo /state_estimator/pose

# Monitor update rate
ros2 topic hz /state_estimator/pose

# Check localization status
ros2 topic echo /localization/status
```

**If Localization Fails:**
1. Verify map is loaded correctly
2. Check initial pose is reasonably accurate
3. Ensure LiDAR data is publishing
4. Verify robot is within mapped area
5. Check for environmental changes since mapping

---

## 13. Troubleshooting

### 13.1 System Won't Start

**Problem:** bringup.launch.py fails

**Diagnosis:**
```bash
# Check which node fails
ros2 launch carver_bringup bringup.launch.py --screen

# Check individual components
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 run carver_odrive carver_odrive.py
```

**Solutions:**

**Micro-ROS Agent Not Connecting:**
```bash
# Check device
ls -l /dev/ttyACM*

# Try different device
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM1

# Check permissions
sudo chmod 666 /dev/ttyACM0
```

**ODrive Not Found:**
```bash
# Check USB
lsusb | grep ODrive

# Try odrivetool
odrivetool

# Check multiple ODrives
# May need to specify serial number in code
```

### 13.2 Localization Failure

**Problem:** Robot pose not converging

**Diagnosis:**
```bash
# Check map loaded
ros2 topic echo /map -n 1

# Check scan data
ros2 topic echo /livox/lidar -n 1

# Monitor convergence
ros2 topic echo /current_pose
```

**Solutions:**

**No Map Data:**
```bash
# Verify map file exists
ls -l /path/to/map.pcd

# Load map manually
ros2 launch lidar_localization_ros2 pcl_localization.launch.py map_path:=/correct/path/map.pcd
```

**Poor Initial Guess:**
```bash
# Provide better initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped ...
```

**Insufficient Features:**
- Drive to area with more geometry
- Check LiDAR is working
- Verify map quality

**NDT Parameters Too Strict:**
```yaml
# Relax parameters in localization.yaml
ndt_resolution: 2.0  # Increase
transformation_epsilon: 0.1  # Increase
```

### 13.3 Controller Not Following Path

**Problem:** Robot doesn't track reference trajectory

**Diagnosis:**
```bash
# Check trajectory published
ros2 topic echo /reference_trajectory

# Check current pose
ros2 topic echo /current_pose

# Check controller output
ros2 topic echo /cmd_vel

# Check controller node
ros2 node info /controller_node
```

**Solutions:**

**No Trajectory:**
```bash
# Publish test trajectory
ros2 topic pub /reference_trajectory nav_msgs/msg/Path ...
```

**Controller Not Running:**
```bash
# Start controller
ros2 run carver_controller carver_stanley.py
```

**Gains Too Low:**
```python
# Increase controller gains
ros2 param set /stanley_controller k_e 1.0
```

**Kinematics Issue:**
- Verify wheelbase in URDF matches physical robot
- Check steering direction (may need to reverse)

### 13.4 Motors Not Responding

**Problem:** ODrive connected but motors don't move

**Diagnosis:**
```python
# In odrivetool
odrv0.axis0.error  # Check for errors
odrv0.axis0.current_state  # Check state
```

**Solutions:**

**Error Code Present:**
```python
# Clear errors
odrv0.clear_errors()

# Check specific error
odrv0.axis0.error  # See ODrive documentation for error codes
```

**Not Calibrated:**
```python
# Run calibration
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

**Wrong Control Mode:**
```python
# Set correct mode
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.save_configuration()
```

**Phase Wires Disconnected/Wrong:**
```
# Check motor phase connections
# Swap two phases if motor vibrates
```

### 13.5 IMU Data Incorrect

**Problem:** IMU readings nonsensical

**Diagnosis:**
```bash
# Check raw data
ros2 topic echo /imu/data

# Check calibration
ros2 topic echo /imu/calibration_status
```

**Solutions:**

**Not Calibrated:**
```
# Perform calibration procedure
# Move IMU through all orientations
# Monitor calibration status until all reach 3
```

**Wrong Orientation:**
```xml
# Check IMU mounting in URDF
<origin xyz="..." rpy="0 0 0"/>  # Adjust rpy to match physical mounting
```

**I2C Communication Issue:**
```bash
# Check I2C on STM32
# Verify pull-up resistors
# Check IMU address (0x28 or 0x29)
```

### 13.6 LiDAR No Data

**Problem:** No point cloud on /livox/lidar topic

**Diagnosis:**
```bash
# Check topic exists
ros2 topic list | grep livox

# Check LiDAR node
ros2 node list | grep livox

# Check network (if Ethernet)
ping 192.168.1.10
```

**Solutions:**

**Network Issue:**
```bash
# Reconfigure network
sudo ifconfig eth0 192.168.1.50 netmask 255.255.255.0

# Check firewall
sudo ufw status
sudo ufw allow from 192.168.1.10
```

**Driver Not Running:**
```bash
# Start Livox driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Power Issue:**
```
# Check 12V supply to LiDAR
# Verify power LED on LiDAR
# Check current draw (~500mA typical)
```

### 13.7 High CPU Usage

**Problem:** System running slow, high CPU usage

**Diagnosis:**
```bash
# Check CPU usage
top
htop

# Check ROS2 nodes
ros2 node list
# Monitor specific node
top -p $(pgrep -f node_name)
```

**Solutions:**

**Reduce Localization Load:**
```yaml
# In localization.yaml
voxel_leaf_size: 0.2  # Increase
localization_frequency: 5  # Decrease
```

**Reduce Point Cloud Density:**
```yaml
# In LiDAR config
point_filter_num: 2  # Increase (skip more points)
```

**Disable Unnecessary Nodes:**
```bash
# Comment out in launch file
# Disable visualization during operation
```

**Optimize Code:**
```python
# Use compiled languages for performance-critical parts
# Profile code to find bottlenecks
```

### 13.8 Build Errors

**Problem:** colcon build fails

**Common Errors and Solutions:**

**Missing Dependencies:**
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**PCL Version Mismatch:**
```bash
sudo apt install libpcl-dev
# Or build specific version from source
```

**Python Module Not Found:**
```bash
pip3 install <module_name>
```

**Compiler Error:**
```bash
# Update compiler
sudo apt update
sudo apt upgrade gcc g++
```

**Clean Build:**
```bash
cd ~/carver_ws
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## 14. Advanced Topics

### 14.1 Custom Controller Development

**Creating New Controller:**

**Step 1: Create Python File**
```python
#!/usr/bin/env python3
# carver_ws/src/carver_controller/scripts/carver_custom.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import numpy as np

class CustomController(Node):
    def __init__(self):
        super().__init__('custom_controller')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/reference_trajectory', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Parameters
        self.declare_parameter('control_gain', 1.0)
        self.gain = self.get_parameter('control_gain').value
        
        # State
        self.reference_path = None
        self.current_pose = None
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def path_callback(self, msg):
        self.reference_path = msg
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def control_loop(self):
        if self.reference_path is None or self.current_pose is None:
            return
            
        # Your control algorithm here
        cmd = self.compute_control()
        self.cmd_pub.publish(cmd)
        
    def compute_control(self):
        cmd = Twist()
        
        # Implement your control law
        # Example: simple proportional control
        target = self.find_closest_point()
        error = self.compute_error(target)
        
        cmd.linear.x = self.gain * error
        cmd.angular.z = self.gain * self.compute_heading_error(target)
        
        return cmd
        
    def find_closest_point(self):
        # Find closest point on path
        pass
        
    def compute_error(self, target):
        # Compute tracking error
        pass
        
    def compute_heading_error(self, target):
        # Compute heading error
        pass

def main(args=None):
    rclpy.init(args=args)
    controller = CustomController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 2: Make Executable**
```bash
chmod +x carver_ws/src/carver_controller/scripts/carver_custom.py
```

**Step 3: Update CMakeLists.txt or setup.py**

For Python nodes in colcon:
```python
# In carver_controller/setup.py
entry_points={
    'console_scripts': [
        'carver_custom = carver_controller.carver_custom:main',
    ],
},
```

**Step 4: Build and Test**
```bash
cd ~/carver_ws
colcon build --packages-select carver_controller
source install/setup.bash
ros2 run carver_controller carver_custom
```

### 14.2 Sensor Fusion

**Combining Multiple Sensors:**

**Example: IMU + Encoder Odometry**

```python
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf_transformations

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribe to sensors
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.encoder_sub = self.create_subscription(...)
        
        # Publish fused estimate
        self.odom_pub = self.create_publisher(Odometry, '/fused_odom', 10)
        
        # Kalman filter or complementary filter
        self.filter = ExtendedKalmanFilter()
        
    def imu_callback(self, msg):
        # Update filter with IMU data
        self.filter.update_imu(msg)
        self.publish_estimate()
        
    def encoder_callback(self, msg):
        # Update filter with encoder data
        self.filter.update_encoder(msg)
        
    def publish_estimate(self):
        # Publish fused estimate
        odom = self.filter.get_estimate()
        self.odom_pub.publish(odom)
```

### 14.3 Path Planning Integration

**Using Nav2 Planner:**

```bash
# Install Nav2
sudo apt install ros-humble-navigation2

# Configure Nav2 for CARVER
# Create config file: nav2_params.yaml
```

**Example Navigation:**
```bash
# Start system
ros2 launch carver_bringup bringup.launch.py

# Start Nav2
ros2 launch nav2_bringup navigation_launch.py params_file:=/path/to/nav2_params.yaml

# Set goal
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped ...
```

### 14.4 Multi-Robot Coordination

**Running Multiple Robots:**

**Robot 1:**
```bash
ros2 launch carver_bringup bringup.launch.py namespace:=robot1
```

**Robot 2:**
```bash
ros2 launch carver_bringup bringup.launch.py namespace:=robot2
```

**Shared Map:**
```bash
# Both robots use same map
# Implement multi-robot SLAM or share map server
```

**Coordination:**
```python
# Implement coordination node
class MultiRobotCoordinator(Node):
    def __init__(self):
        # Subscribe to all robot poses
        # Implement collision avoidance
        # Assign tasks
        pass
```

### 14.5 Data Logging and Analysis

**Recording Data:**
```bash
# Record all topics
ros2 bag record -a -o carver_experiment

# Record specific topics
ros2 bag record /cmd_vel /odom /imu/data /livox/lidar
```

**Playback:**
```bash
ros2 bag play carver_experiment_0.db3
```

**Analysis:**

```python
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt

# Read bag file
conn = sqlite3.connect('carver_experiment_0.db3')
topics = pd.read_sql_query("SELECT * FROM topics", conn)
messages = pd.read_sql_query("SELECT * FROM messages", conn)

# Parse and plot
# (Requires deserialization of messages)
```

**Tools:**
- `ros2 bag`
- `PlotJuggler`
- `rqt_bag`
- Custom Python scripts

### 14.6 Simulation to Real Transfer

**Techniques:**

**1. Model Fidelity**
```xml
<!-- Accurate inertia in URDF -->
<inertial>
  <mass value="30.0"/>  <!-- Measured -->
  <inertia ixx="..." iyy="..." izz="..."/>  <!-- Calculated or measured -->
</inertial>
```

**2. Sensor Noise Models**
```python
# Add realistic noise in simulation
imu_noise = np.random.normal(0, 0.01, size=3)
encoder_noise = np.random.normal(0, 0.0001)
```

**3. Domain Randomization**
- Vary simulation parameters
- Different lighting, textures
- Varied dynamics

**4. Gradual Deployment**
1. Test in simulation
2. Test in controlled environment
3. Test in target environment
4. Deploy

### 14.7 Performance Benchmarking

**Metrics:**

**Localization:**
- Pose error (m, degrees)
- Convergence time (s)
- Computational load (%)
- Update rate (Hz)

**Control:**
- Tracking error (m)
- Smoothness (jerk m/s³)
- Energy consumption (Wh)
- Task completion time (s)

**Benchmarking Script:**
```python
class PerformanceMonitor(Node):
    def __init__(self):
        self.start_time = time.time()
        self.errors = []
        
        # Subscribe to relevant topics
        self.pose_sub = self.create_subscription(...)
        
    def compute_metrics(self):
        mean_error = np.mean(self.errors)
        std_error = np.std(self.errors)
        max_error = np.max(self.errors)
        
        self.get_logger().info(f'Mean error: {mean_error:.3f}m')
        self.get_logger().info(f'Std error: {std_error:.3f}m')
        self.get_logger().info(f'Max error: {max_error:.3f}m')
```

---

## 15. Development Guidelines

### 15.1 Code Structure

**Package Organization:**
```
carver_package/
├── carver_package/       # Python package
│   ├── __init__.py
│   └── module.py
├── include/              # C++ headers
│   └── carver_package/
│       └── header.hpp
├── src/                  # C++ source
│   └── node.cpp
├── scripts/              # Executable scripts
│   └── node.py
├── launch/               # Launch files
├── config/               # Configuration files
├── msg/                  # Custom messages
├── srv/                  # Custom services
├── CMakeLists.txt        # Build configuration
├── package.xml           # Package metadata
├── setup.py              # Python setup
└── README.md             # Package documentation
```

### 15.2 Coding Standards

**Python:**
- Follow PEP 8
- Use type hints
- Document with docstrings
- Use meaningful variable names

**Example:**
```python
def compute_steering_angle(
    cross_track_error: float,
    heading_error: float,
    velocity: float,
    gain: float = 0.5
) -> float:
    """
    Compute steering angle using Stanley method.
    
    Args:
        cross_track_error: Lateral error in meters
        heading_error: Angular error in radians
        velocity: Current velocity in m/s
        gain: Control gain (default 0.5)
        
    Returns:
        Steering angle in radians
    """
    steering = heading_error + np.arctan(gain * cross_track_error / (1.0 + velocity))
    return np.clip(steering, -MAX_STEERING, MAX_STEERING)
```

**C++:**
- Follow ROS2 style guide
- Use modern C++17 features
- RAII principles
- Clear ownership

### 15.3 Testing

**Unit Tests:**

```python
# test/test_controller.py
import unittest
from carver_controller.stanley import StanleyController

class TestStanleyController(unittest.TestCase):
    def setUp(self):
        self.controller = StanleyController()
        
    def test_steering_computation(self):
        result = self.controller.compute_steering(
            cross_track_error=1.0,
            heading_error=0.1,
            velocity=2.0
        )
        self.assertIsInstance(result, float)
        self.assertGreater(result, -1.0)
        self.assertLess(result, 1.0)
```

**Run Tests:**
```bash
cd ~/carver_ws
colcon test
colcon test-result --verbose
```

### 15.4 Documentation

**Package README:**

Each package should have README.md:
```markdown
# Package Name

## Overview
Brief description

## Nodes
List of nodes

## Topics
Subscribed and published topics

## Parameters
Configuration parameters

## Usage
Examples

## Dependencies
List dependencies
```

**Code Documentation:**
- Comment complex algorithms
- Explain parameter choices
- Document assumptions
- Note known limitations

```

## 16. References

### 16.1 Software Documentation

**ROS2:**
- ROS2 Documentation: https://docs.ros.org/en/humble/
- ROS2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html

**Libraries:**
- Point Cloud Library: https://pointclouds.org/documentation/
- OpenCV: https://docs.opencv.org/
- Eigen: https://eigen.tuxfamily.org/

**Hardware:**
- ODrive Documentation: https://docs.odriverobotics.com/
- Livox SDK: https://github.com/Livox-SDK/
- BNO055 Datasheet: Bosch Sensortec

### 16.2 Academic Papers

**SLAM:**
- Fast-LIO: https://github.com/hku-mars/FAST_LIO

**Control:**
- Stanley: "Stanley: The Robot that Won the DARPA Grand Challenge"
- Pure Pursuit: "Implementation of the Pure Pursuit Path Tracking Algorithm"
- MPC: "Model Predictive Control: Theory and Design"

### 16.3 Related Projects

- Autoware: https://www.autoware.org/
- ROS2 Navigation: https://navigation.ros.org/
- Apollo: https://github.com/ApolloAuto/apollo

### 16.4 Community

**Forums:**
- ROS Discourse: https://discourse.ros.org/
- ROS Answers: https://answers.ros.org/
- Robotics Stack Exchange: https://robotics.stackexchange.com/

**GitHub:**
- Issues: Report bugs and request features
- Discussions: Ask questions and share ideas
- Pull Requests: Contribute improvements

---

## Appendix

### A. Glossary

**Terms:**
- **NDT:** Normal Distributions Transform
- **MPC:** Model Predictive Control
- **SLAM:** Simultaneous Localization and Mapping
- **ODrive:** Open-source motor controller
- **Micro-ROS:** ROS2 for microcontrollers
- **PCL:** Point Cloud Library
- **TF:** Transform library in ROS
- **URDF:** Unified Robot Description Format

### B. Keyboard Shortcuts

**Manual Control:**
- W: Forward
- S: Backward
- A: Steer left
- D: Steer right
- Space: Emergency stop
- ESC: Exit

**RViz:**
- Mouse wheel: Zoom
- Middle click + drag: Pan
- Right click + drag: Rotate view

### C. Useful Commands

**Quick Reference:**
```bash
# List nodes
ros2 node list

# Check topic
ros2 topic echo /topic_name

# Check TF
ros2 run tf2_tools view_frames

# Monitor CPU
htop

# Check logs
ros2 run rqt_console rqt_console

# Restart node
ros2 lifecycle set /node_name configure
```

---

**Document Version:** 1.0  
**Last Updated:** December 2025  
**Maintained by:** CARVER Development Team