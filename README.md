# CARVER-GEN3 Autonomous Robot Platform

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?style=flat-square&logo=ros" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/Ubuntu-22.04-orange?style=flat-square&logo=ubuntu" alt="Ubuntu 22.04">
  <img src="https://img.shields.io/badge/Python-3.10-yellow?style=flat-square&logo=python" alt="Python 3.10">
  <img src="https://img.shields.io/badge/MOLA-SLAM-green?style=flat-square" alt="MOLA SLAM">
  <img src="https://img.shields.io/badge/LiDAR-Livox%20MID360-red?style=flat-square" alt="Livox MID360">
  <img src="https://img.shields.io/badge/MCU-STM32G474RE-blue?style=flat-square&logo=stmicroelectronics" alt="STM32G474RE">
  <img src="https://img.shields.io/badge/License-MIT-brightgreen?style=flat-square" alt="License MIT">
  <img src="https://img.shields.io/badge/Drive-Rear%20Wheel-lightgrey?style=flat-square" alt="Rear Wheel Drive">
  <img src="https://img.shields.io/badge/Steering-Ackermann-lightblue?style=flat-square" alt="Ackermann Steering">
</p>



## Overview

CARVER-GEN3 is an autonomous mobile robot platform with rear-wheel drive, Ackermann steering, and LiDAR-based navigation. Built on ROS2 Humble for research in autonomous navigation, SLAM, and path tracking control.

**Version:** 3.0  
**ROS2 Distribution:** Humble  
**Operating System:** Ubuntu 22.04 LTS  
**License:** MIT  

<p align="center">
  <a href="media/demo.mp4">
    <img src="media/demo.webp" alt="Demo Video">
  </a>
</p>

---

## Table of Contents

1. [Key Features](#1-key-features)
2. [System Architecture](#2-system-architecture)
3. [Hardware Requirements](#3-hardware-requirements)
4. [Installation](#4-installation)
5. [Package Descriptions](#5-package-descriptions)
6. [Additional Hardware Connections](#6-additional-hardware-connections)
7. [Configuration](#7-configuration)
8. [Quick Start Guide](#8-quick-start-guide)
9. [Path Tracking Controllers](#9-path-tracking-controllers)
10. [Localization and Mapping](#10-localization-and-mapping)
11. [Troubleshooting](#11-troubleshooting)
12. [References](#12-references)

---

## 1. Key Features

### Hardware
- **Drive System:** Rear-wheel drive (Using ODrive controllers)
- **Steering:** Front Ackermann steering with absolute encoders
- **Sensing:** Livox MID360 LiDAR, BNO055 IMU, AMT212EV encoders
- **Control:** 3× STM32G474RE microcontrollers running Micro-ROS

### Software
- **SLAM:** MOLA framework for mapping ([Complete MOLA guide here](https://github.com/Whan000/MOLA-SLAM))
- **Localization:** GICP/ICP Cov matching
- **Controllers:** Stanley, Pure Pursuit and Combined path tracking
- **Real-time Control:** 10 Hz control loop, 10 Hz localization

### Specifications
- **Wheelbase:** 0.8m
- **Max Speed:** 3.5 m/s
- **Steering Range:** ±0.6 rad (±34°)
- **Localization Accuracy:** <0.02m

---

## 2. System Architecture

![System Architecture](media/System%20Architecture.png)

### System Overview

The Carver system consists of three main subsystems: **Perception & Localization**, **Decision & Control**, and **Hardware Interface (Micro-ROS)**. The system uses pre-planned paths with MOLA-SLAM for localization, enabling autonomous path following.

---

### 1. Hardware Interface Layer (Micro-ROS Agent)

The `micro_ros_agent` bridges three STM32 microcontrollers to ROS2:

- `bno055_publisher` - Reads BNO055 IMU data from STM32
  - Publishes: `/bno055_data` (Float64MultiArray)

- `carver_steering` - Controls steering servo, reads AMT212 encoder feedback
  - Subscribes: `/steering_angle` (Float32)
  - Publishes: `/amt_publisher` (amt_read)

- `carver_interface` - Handles accelerator input, emergency button, and lighting control
  - Subscribes: `/light_signal_command` (Int8)
  - Publishes: `/accl_publisher` (UInt16), `/carver_emergency` (Bool), `/accel_direction` (Int8), `/carver_mode` (Int8)

---

### 2. Perception & Localization

**Sensors:**
- `carver_livox_lidar.launch.py` - Launches Livox 3D LiDAR
  - Publishes: `/livox/lidar` (PointCloud2), `/livox/imu` (Imu)

- `bno055_imu.py` - Processes raw IMU data from STM32
  - Subscribes: `/bno055_data`
  - Publishes: `/imu` (sensor_msgs/Imu)

**Localization:**
- `MOLA_lidar_localization` - 3D LiDAR-based localization using pre-built maps
  - Subscribes: `/livox/lidar`, `/imu`
  - Publishes: `/state_estimator/pose` (nav_msgs/Odometry)
  - Requires: `map.simplemap` + `map.mm` (pre-built map files)

---

### 3. Decision & Control Layer

**Mode Management:**
- `carver_mode.py` - Switches modes
  - Subscribes: `/controller_speed`, `/manual_speed`
  - Publishes: `/target_speed`, `/carver_mode`

**Path Tracking Controllers:**
- `carver_controller.py` - Implements Stanley / Pure Pursuit / Combined controller
  - Subscribes: `/state_estimator/pose`
  - Publishes: `/controller_speed`, `/steering_angle`
  - Requires: `path.yaml` (pre-planned waypoints)

**Manual Control:**
- `carver_manual_steering.py` - Manual joystick/handle control
  - Subscribes: `/accel_direction`, `/accl_publisher`
  - Publishes: `/manual_speed`

**Motor Control:**
- `carver_odrive.py` - Interfaces with ODrive motor controller
  - Subscribes: `/target_speed`
  - Outputs: Velocity setpoint to ODrive (with velocity feedback)

---

### Operational Sequence

**Phase 1: Mapping (Offline)**
1. Drive the vehicle manually while recording LiDAR/IMU data
2. Build `map.simplemap` and `map.mm` using MOLA offline tools
3. Define waypoints and save as `path.yaml`

**Phase 2: Localization (Runtime)**
1. Livox LiDAR publishes point cloud to `/livox/lidar`
2. BNO055 IMU data flows: STM32 → micro_ros → `bno055_imu.py` → `/imu`
3. MOLA localizes against pre-built map, publishes pose to `/state_estimator/pose`

**Phase 3: Path Following (Runtime)**
1. Controller loads waypoints from `path.yaml`
2. Controller receives current pose from `/state_estimator/pose`
3. Stanley/Pure Pursuit algorithm computes:
   - Steering angle → `/steering_angle` → STM32 → Servo
   - Target speed → `/controller_speed` → `carver_mode` → `/target_speed` → ODrive

**Phase 4: Actuation**
1. `carver_steering` (STM32) receives steering command, controls servo via PWM
2. AMT212 encoder provides steering angle feedback
3. `carver_odrive` sends velocity setpoint to ODrive S1 Pro
4. ODrive returns velocity feedback for closed-loop control


---

## 3. Hardware Requirements

### Computing Hardware

**Minimum Requirements:**
- CPU: Quad-core x86-64 @ 2.0 GHz
- RAM: 16 GB DDR5
- Storage: 256 GB SSD
- OS: Ubuntu 22.04 LTS
- USB: Multiple USB 3.0 ports

**Recommended (Tested Configuration):**
- **System:** ASUS ROG G17 (2025)
- **CPU:** AMD Ryzen 9 7945HX (16 cores, 32 threads)
- **GPU:** NVIDIA RTX 4060 (optional, for future vision features)
- **RAM:** 24 GB DDR5
- **Storage:** NVMe SSD 2TB

### Microcontrollers

**3× STM32G474RE Units:**
- ARM Cortex-M4F @ 170MHz
- 512KB Flash, 128KB RAM
- Hardware FPU, USB 2.0
- Running Micro-ROS firmware

**Unit 1:** Encoder interface (AMT212EV)
**Unit 2:** Steering control  
**Unit 3:** BNO055 IMU interface

### Sensors

**Livox MID360 LiDAR:**
- 360° horizontal FOV
- Up to 70m range
- 200K points/sec
- Connection: Ethernet or USB
- Power: 12V DC

**BNO055 9-DOF IMU:**
- Built-in sensor fusion
- Connection: I2C to STM32
- Power: 3.3V or 5V

**AMT212EV Absolute Encoders:**
- 12-bit resolution (4096 positions/rev)
- Interface: SPI
- Power: 5V DC

### Actuators

**ODrive v3.6 Motor Controllers:**
- Connection: USB 2.0
- Power: 24V DC
- Hall sensor feedback

**Motors:**
- Brushless DC motors (rear wheels)
- Built-in Hall sensors
- Direct drive

**Front Steering:**
- DC Motor actuated
- Ackermann geometry linkage
- AMT212EV encoders for feedback

---

## 4. Installation

### Prerequisites

- **Ubuntu 22.04 LTS**
- **ROS2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **rosdep initialized** 
```bash
sudo rosdep init && rosdep update
```

### Clone Repository
```bash
git clone https://github.com/Whan000/CARVER-GEN3.git
cd CARVER-GEN3/carver_ws
```

### Install Dependencies
```bash
# Point Cloud Library
sudo apt install libpcl-dev ros-humble-pcl-ros ros-humble-pcl-conversions pcl-tools

# Python dependencies
pip3 install numpy scipy matplotlib transforms3d pyserial pyyaml odrive

# Clone Livox driver into src/
git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2

# Install ROS dependencies via rosdep
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
echo "source ~/CARVER-GEN3/carver_ws/install/setup.bash" >> ~/.bashrc
```

### Micro-ROS Setup
```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# Build firmware and agent
ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

> **STM32/Nucleo Setup:** See [Micro-ROS with Nucleo Board](https://github.com/pboon09/Micro-ROS-with-Nucleo-board) for firmware flashing and configuration.

### Device Permissions
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# ODrive udev rules
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0d32", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/91-odrive.rules

sudo udevadm control --reload-rules && sudo udevadm trigger
# Logout/login required
```

### Verify Installation
```bash
ros2 pkg list
```

Expected packages:
- amt212ev_interfaces
- bno055_imu
- carver_bringup
- carver_controller
- carver_description
- carver_manager
- carver_odrive
- carver_simulation
- lidar_localization_ros2
- ndt_omp_ros2

---

## 5. Package Descriptions

### 5.1 carver_description

URDF robot model and visualization.

**Contents:**
- `urdf/` - Robot structure definition
- `meshes/` - STL files
- `launch/` - Visualization launch files
- `config/` - Controller parameters

**Usage:**
```bash
ros2 launch carver_description simple_display.launch.py
```

### 5.2 carver_bringup

System startup launch files.

**Launch Files:**
- `bringup.launch.py` - Full system startup
- `manual_steering.launch.py` - Manual control only
- `slam.launch.py` - SLAM / mapping mode (see [MOLA-SLAM](https://github.com/Whan000/MOLA-SLAM) for details)
- `uros.launch.py` - Micro-ROS agents only

**Usage:**
```bash
ros2 launch carver_bringup bringup.launch.py
```

### 5.3 carver_controller

Path tracking controllers.

**Controllers:**
- `carver_stanley.py` - Stanley Controller
- `carver_purepursuit.py` - Pure Pursuit Controller
- `carver_combined.py` - Adaptive Combined Controller

**Trajectory Files:**
- `path/trajectory200.yaml` - 200 path sampling points
- `path/trajectory1000.yaml` - 1000 path sampling points

### 5.4 carver_odrive

ODrive motor controller interface.

**Scripts:**
- `carver_odrive.py` - Main interface node
- `dummy_steering.py` - Testing/simulation

### 5.5 carver_manager

Mode management and manual control.

**Scripts:**
- `carver_mode.py` - Mode switching
- `carver_manual_steering.py` - Manual control

### 5.6 carver_simulation

Gazebo simulation environment.

**Usage:**
```bash
ros2 launch carver_simulation simulation-full.launch.py
```

### 5.7 bno055_imu

BNO055 IMU data converter (Micro-ROS → standard ROS2).

### 5.8 lidar_localization_ros2

NDT-OMP based localization.

**Features:**
- Real-time point cloud registration
- Map loading and management
- Pose estimation

### 5.9 ndt_omp_ros2

OpenMP-accelerated NDT library.

---

## 6. Additional Hardware Connections

### Livox MID360

**Ethernet Connection:**
```bash
# Configure static IP
sudo ifconfig eth0 192.168.1.50 netmask 255.255.255.0

# Test connection
ping 192.168.1.10
```


### Persistent Device Names

**Create udev rules:**

```bash
sudo nano /etc/udev/rules.d/99-carver-usb.rules
```

**Add rules (adjust serial numbers to match your devices):**

```bash
# Encoder Interface
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="ENCODER_UNIT_001", SYMLINK+="carver_interface", \
  MODE="0666", GROUP="dialout"

# Steering Control
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="STEERING_UNIT_001", SYMLINK+="carver_steering", \
  MODE="0666", GROUP="dialout"

# IMU
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
  ATTRS{serial}=="IMU_UNIT_001", SYMLINK+="carver_bno055", \
  MODE="0666", GROUP="dialout"
```

**Reload rules:**
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

**Verify:**
```bash
ls -l /dev/carver_*
```

**Always use symbolic names in code:**
```python
# ✓ Good
serial_port = '/dev/carver_interface'

# ✗ Bad - will break!
serial_port = '/dev/ttyACM1'
```

---

## 7. Configuration

### Robot Parameters

**File:** `carver_description/urdf/carver_params.xacro`

```xml
<!-- Physical dimensions -->
<xacro:property name="wheelbase_length" value="0.8"/>  <!-- meters -->
<xacro:property name="track_width" value="0.4"/>
<xacro:property name="wheel_radius" value="0.075"/>

<!-- Steering limits -->
<xacro:property name="max_steering_angle" value="0.6"/>  <!-- ~34° -->

<!-- Mass properties -->
<xacro:property name="base_mass" value="25.0"/>  <!-- kg -->
```


### ODrive Configuration

**File:** `odrive_config/OdriveConfig.json`

**Key Parameters:**
```json
{
  "axis0.config.motor.pole_pairs": 4,
  "axis0.config.motor.phase_resistance": 0.0354,
  "axis0.config.motor.current_soft_max": 50.0,
  "axis0.config.motor.current_hard_max": 100.0,
  
  "axis0.controller.config.control_mode": 2,  // Velocity
  "axis0.controller.config.vel_limit": 60.0,  // Rev/s
  "axis0.controller.config.vel_gain": 3.0,
  
  "hall_encoder0.config.enabled": true,
  "hall_encoder0.config.edges_calibrated": true
}
```


**Calibrate each ODrive:**
```python
# In odrivetool
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
# Wait for completion
odrv0.save_configuration()
odrv0.reboot()
```

### Controller Parameters

**File:** `carver_description/config/carver_controller_param.yaml`

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500  # Hz
    use_sim_time: true

steering_controller:
  ros__parameters:
    joints:
      - base_front_left_steering_joint
      - base_front_right_steering_joint

wheel_velocity_controller:
  ros__parameters:
    joints:
      - base_rear_left_wheel_joint
      - base_rear_right_wheel_joint
```
---

## 8. Quick Start Guide

### Create Map with MOLA

**MOLA SLAM** is used to create maps for autonomous navigation.

**Complete MOLA configuration and usage guide:**  
**https://github.com/Whan000/MOLA-SLAM**

### Quick Overview:

**1. Start System:**
```bash
# Terminal 1
ros2 launch carver_bringup bringup.launch.py

# Terminal 2 - Localization
ros2 launch mola_bringup mola_slam.py
```

**2. Set Initial Pose:**
1. mola_localize_launch.py will show a GUI
2. Key your initial pose

**3. Start Controller:**

**Stanley:**
default:
```bash
ros2 run carver_controller carver_stanley.py 
```

**Pure Pursuit:**
default:
```bash
ros2 run carver_controller carver_purepursuit.py 
```

**Combined:**
default:
```bash
ros2 run carver_controller carver_combined.py
```

**4. Emergency Stop:**

Press hardware E-stop

---

## 9. Path Tracking Controllers

### Stanley Controller

The **Stanley Controller** uses a non-linear control law to keep the vehicle's front axle on the path. It combines heading error ($\theta_e$) and cross-track error ($e$).

**Topics:**

Subscribed:

  - `/state_estimator/pose` (nav\_msgs/Odometry)
  - `/imu` (sensor\_msgs/Imu)

Published:

  - `/steering_angle` (std\_msgs/Float32)
  - `/controller_speed` (std\_msgs/Float32)
  - `/path_visualization` (nav\_msgs/Path)

**Parameters:**

```yaml
k_heading: 0.25          # Heading error gain (Implicit in control law)
k_crosstrack: 0.1        # Cross-track error gain (k)
ks_gain: 0.1             # Velocity softening factor (k_s, for denominator)
target_speed: 1.25       # m/s
max_steer: 0.6           # rad (~34°)
lookahead_distance: 5.0  # m (Note: Stanley uses *closest point* for CTE, not lookahead distance)
```

**Steering Law:**
The control law calculates the required steering angle ($\delta$) based on the heading error ($\theta_e$) and the cross-track error ($e$).

$$
\delta = \theta_e + \arctan\left(\frac{k_{\text{crosstrack}} \cdot e}{k_s + v}\right)
$$

Where:

  * $\delta$: steering angle
  * $\theta_e$: heading error (orientation difference between vehicle and path segment)
  * $e$: cross-track error (perpendicular distance from front axle to path)
  * $v$: current longitudinal velocity
  * $k_{\text{crosstrack}}$: proportional gain for correcting lateral error
  * $k_s$: speed softening constant to prevent singularity at $v \approx 0$

**Tuning:**

  * $\uparrow$ `k_heading` = more aggressive heading correction
  * $\uparrow$ `k_crosstrack` = faster lateral correction
  * $\uparrow$ `lookahead_distance` = smoother but may cut corners

**Best for:**

  * Preplanned trajectories
  * Racing or time trials

-----

### Pure Pursuit Controller

The **Pure Pursuit Controller** uses geometric control based on driving the vehicle towards a single target point located some distance ($l_d$) ahead on the path.

**Topics:**

Subscribed:

  - `/state_estimator/pose` (nav\_msgs/Odometry)
  - `/imu` (sensor\_msgs/Imu)

Published:

  - `/steering_angle` (std\_msgs/Float32)
  - `/controller_speed` (std\_msgs/Float32)
  - `/path_visualization` (nav\_msgs/Path)

**Parameters:**

```yaml
wheelbase: 0.8               # m (L in the formula)
max_steering_angle: 0.6      # rad
min_lookahead: 3.0           # m (Ld minimum)
max_lookahead: 5.0           # m (Ld maximum)
lookahead_gain: 1.0          # Speed-dependent gain (k_l in dynamic Ld formula)
target_speed: 1.0            # m/s
```

**Steering Law:**
The required steering angle ($\delta$) is calculated to reach the lookahead point, forming an arc based on Ackermann kinematics:

$$
\delta = \arctan\left(\frac{2 \cdot L \cdot \sin(\alpha)}{l_d}\right)
$$

Where:

  * $L$: wheelbase
  * $\alpha$: angle between the vehicle's current heading and the vector to the lookahead point
  * $l_d$: lookahead distance

**Dynamic Lookahead:**
The lookahead distance is often made proportional to the vehicle's speed for smooth performance:

$$
l_d = \min_{\text{lookahead}} + \text{lookahead}_{\text{gain}} \times v
$$

$$
l_d = \text{clip}(l_d, \min_{\text{lookahead}}, \max_{\text{lookahead}})
$$

**Features:**

  - Automatic speed reduction in turns
  - Monotonic waypoint progression
  - Completion detection

**Best for:**

  - Smooth curved paths
  - Initial testing

-----

### Combined Controller

The **Combined Controller** dynamically blends the outputs of the Pure Pursuit ($\delta_{PP}$) and Stanley ($\delta_{ST}$) controllers based on the **curvature of the path** ($\gamma$) at the robot's location. This aims to leverage the stability of Pure Pursuit on curves and the accuracy of Stanley on straights.

**Algorithm Logic:**

1.  **Calculate Path Curvature ($\gamma$):** Determine the path curvature at the current segment using the angle change between adjacent path vectors.

2.  **Normalize Curvature ($\gamma_{\text{norm}}$):** $\gamma$ is normalized relative to a minimum turning radius constraint ($\gamma_{\text{norm}}$).

$$
\gamma_{\text{norm}} = 2 \cdot \arcsin\left(\frac{d}{2 \cdot R_{\min}}\right)
$$

3.  **Calculate Weight Factors ($\mathbf{k_{PP}}, \mathbf{k_{ST}}$):** The weighting factors depend on the normalized curvature, varying between $K_{\min}$ (for straight sections) and $K_{\max}$ (for sharp turns).

$$
k_{PP} = K_{\min} + \left(\frac{\gamma}{\gamma_{\text{norm}}}\right) \cdot (K_{\max} - K_{\min})
$$

$$
k_{ST} = 1 - k_{PP}
$$

4.  **Combine Outputs:** The final steering command ($\delta$) is a weighted average.

$$
\delta = k_{PP} \cdot \delta_{PP} + k_{ST} \cdot \delta_{ST}
$$

**Control Law:**

$$
\delta = \left[K_{\min} + \left(\frac{\gamma}{\gamma_{\text{norm}}}\right) \cdot (K_{\max} - K_{\min})\right] \cdot \delta_{PP} + \left[1 - k_{PP}\right] \cdot \delta_{ST}
$$

**Tuning Parameters (Embedded in Code):**

  * **Weights:** $K_{\min} (0.2)$ and $K_{\max} (0.8)$ define the range of Pure Pursuit influence.
  * **Curvature Norm ($\gamma_{\text{norm}}$):** Derived from the minimum turning radius ($R_{\min} = 7.0\text{ m}$).
  * **Gains:** Inherits tuning properties from both Stanley and Pure Pursuit (e.g., `k_crosstrack`, `lookahead_gain_k1`).

### Creating Trajectories

**YAML Format:**
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
```

---

## 10. Localization and Mapping

### System Overview

**Two-Stage Approach:**

1. **SLAM (Mapping):** Use MOLA to create maps
2. **Localization (Navigation):** Use ICP/GICP for real-time pose estimation → Configured in MOLA packages

**Complete Documentation:**  
**https://github.com/Whan000/MOLA-SLAM**

**For details on:**
- MOLA installation and setup
- Configuration parameters
- Mapping best practices
- Troubleshooting
- Advanced features

**See the dedicated MOLA-SLAM repository above.**

---

## 11. Troubleshooting

### System Won't Start

**Check devices:**
```bash
ls -l /dev/ttyACM*
lsusb | grep ODrive

#Or just go to your browser and type for odriveGUI
```

**Test individual components:**
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
ros2 run carver_odrive carver_odrive.py
```

**Fix permissions:**
```bash
sudo chmod 666 /dev/ttyACM*
```

### Localization Failure

**Verify map loaded:**
```bash
#Check on RViZ2 or MOLA-Viz
```

**Check LiDAR data:**
```bash
ros2 topic echo /livox/lidar -n 1
```

### Motors Not Responding

**Check ODrive:**
```python
# In odrivetool
odrv0.axis0.error
odrv0.clear_errors()
```

**Run calibration:**
```python
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
```

**Check control mode:**
```python
odrv0.axis0.controller.config.control_mode = 2  # Velocity
odrv0.save_configuration()
```

### Controller Not Following Path

**Check pose:**
```bash
ros2 topic echo /state_estimator/pose
```

**Adjust gains:**
```bash
ros2 param set /stanley_controller k_heading 0.5
```

### LiDAR No Data

**Network (Ethernet):**
```bash
ping 192.168.1.10
sudo ifconfig eth0 192.168.1.50 netmask 255.255.255.0
```

**Start driver:**
```bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

### High CPU Usage

**Reduce load:**
```yaml
#Config in MOLA pipelines aim for bigger voxels size
```

**Disable visualization:**
```bash
# Comment out RViz in launch file
```

### Build Errors

**Update dependencies:**
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Clean build:**
```bash
cd ~/carver_ws
rm -rf build/ install/ log/
colcon build
```

---

## 12. References

### Project Repositories

**CARVER-GEN3 Main Repository:**
- **https://github.com/Whan000/CARVER-GEN3**

**MOLA SLAM Configuration (Complete Guide):**
- **https://github.com/Whan000/MOLA-SLAM**
- Installation, setup, parameters, mapping procedures
- Tested configuration for Livox MID360
- Troubleshooting and best practices

**Livox MID360 Guidelines:**
- **https://github.com/Whan000/Livox-MID360**
- Installation, setup, parameters

### Official Documentation

**ROS2:**
- https://docs.ros.org/en/humble/
- https://docs.ros.org/en/humble/Tutorials.html

**MOLA (Upstream):**
- https://github.com/MOLAorg/mola
- https://docs.mola-slam.org/

**Hardware:**
- ODrive: https://docs.odriverobotics.com/
- Livox SDK: https://github.com/Livox-SDK/
- STM32: https://www.st.com/


## Contributors

This project exists thanks to all the people who contributed.

<a href="https://github.com/Whan000/CARVER-GEN3/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Whan000/CARVER-GEN3&t=1" />
</a>

### Academic Papers

- **MOLA:** "A Modular Optimization Framework for Localization and Mapping" (RSS 2019)
- **Stanley Controller:** "Stanley: The Robot that Won the DARPA Grand Challenge"
- **Pure Pursuit Controller:** "Implementation of the Pure Pursuit Path Tracking Algorithm"
- **Combined Controller:** "Combined Path Following Controller for Autonomous Vehicles"
