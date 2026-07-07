# G1Pilot

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS2-Humble-green)](
https://docs.ros.org/en/humble/index.html)
[![GitHub Stars](https://img.shields.io/github/stars/Hucebot/g1pilot?style=social)](https://github.com/Hucebot/g1pilot/stargazers)

<img src="https://github.com/hucebot/g1pilot/blob/main/images/g1pilot.png" alt="G1Pilot" width="800" height="500">

G1Pilot is an open‑source ROS 2 package for Unitree G1 humanoid robots. Basically is made to leave the robot lower body to the controller of unitree while providing all necessary tools to control the upper body and teleoperate the robot. It exposes two complementary control Joint (low‑level, per‑joint) and Cartesian (end‑effector) and continuously publishes core robot state for monitoring and visualization in RViz.

## Highlights

- Dual controller: Unitree’s built‑in loco controller for walking + custom upper‑body controller for arm manipulation.

- Dual control modes: switch between Joint and Cartesian control on the fly.

- Always‑on telemetry: IMU, odometry, and per‑motor feedback (temperature, voltage, position, velocity).

- RViz‑ready: packaged URDF + RViz config for immediate visualization of the real robot.

- Docker‑first workflow: reproducible build/run scripts for Ubuntu 22.04 + ROS 2 Humble.

- Extensible: clear node boundaries and parameters make it easy to add behaviors or swap planners.

- Navigation stack integrated: MOLA odometry and path planner for autonomous navigation.

## G1Pilot Flow

<img src="https://github.com/hucebot/g1pilot/blob/main/images/g1pilot_flow.jpg" alt="G1Pilot Flow" width="800">

## G1Pilot Features


| **Joint Controller** | **Cartesian Controller** |
|---------------------|--------------------|
| <img src="https://github.com/hucebot/g1pilot/blob/main/images/joint_controller.gif" alt="Static Sensors" width="380"> | <img src="https://github.com/hucebot/g1pilot/blob/main/images/cartesian_controller.gif" alt="Moving Sensors" width="380"> |
| **Path Planner & Odometry** | **Control Interface** |
| <img src="https://github.com/hucebot/g1pilot/blob/main/images/odometry_and_pathplanner.gif" alt="Path Planner" width="380"> | <img src="https://github.com/hucebot/g1pilot/blob/main/images/control_interface.gif" alt="Control Interface" width="380">  |

## Table of Contents
- [Pre-requisites](#pre-requisites)
- [Quick Start](#quick-start)
- [Nodes Overview](#-nodes-overview)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Pre-requisites
- Be connected to the robot via Ethernet. **It's important to know which interface you are using.**

## Quick Start
### Docker (recommended)
We prepare two docker images to build and run the package. One is for building in the teleoperation station, and the other is for running in the robot. Both images
are located in the `docker` folder. You can build and run the images with the provided scripts.
To build the docker image in the laptop, run the following command:
  ```bash
  sh build.sh
  ```

To build the docker image in the robot, run the following command:
  ```bash
  sh build_camera.sh
  ```

Then, you can run the docker image in the laptop with the following command:
  ```bash
  sh run.sh
  ```

To run the docker image in the robot with the following command:
  ```bash
  sh run_camera.sh
  ```

## 🧠 Nodes Overview

- **robot_state**: Publishes the state of the robot, including joint positions, velocities, and efforts and custom message to visualize the temperature and voltage of each motor.
- **interactive_marker**: Provides an interactive marker in RViz to control the end-effector position and orientation in Cartesian space.
- **inspire_hand**: Inspire RH56DFTP-2 hand bridge for the MuJoCo sim. Serves the HTML control/force GUIs over WebSocket (`:8766` / `:8765`), drives the URDF finger joints via `/joint_states` (RViz), and (backend `mujoco`) exchanges `rt/inspire/cmd|state` with the sim for real finger angles + touch forces. Open/close is also reachable via `/g1pilot/hand_action/{left,right}` (Streamdeck / loco_client). See [`g1pilot/manipulation/inspire_ftp/README.md`](g1pilot/manipulation/inspire_ftp/README.md). Toggle with `G1_INSPIRE_HANDS` in `start.sh`.
- **joystick**: Node to teleoperate the robot using a joystick, mapping joystick inputs to robot commands.
- **joy_mux**: Multiplexer for joystick inputs, allowing to switch between different control modes, specifcally made to provide autonomous navigation and teleoperation using the same joystick.
- **loco_client**: Client node to communicate with the Unitree loco controller, providing high-level commands for walking and balancing and low-level commands for joint control and cartesian control.
- **dijkstra_planner**: Custom path planner using Dijkstra's algorithm to compute optimal paths for the robot to follow in a given environment with a look ahead distance parameter to smooth the path and improve navigation performance.
- **nav2point**: Node to integrate the planner with the navigation stack, converting navigation goals into waypoints for the robot to follow.
- **create_map**: Dummy node to create a 2D occupancy grid map from the robot's sensors, used for navigation and obstacle avoidance. 
- **mola_fixed**: Node to interface with the MOLA odometry system, transform the odometry data into g1 frame.
- **arm_controller**: Node to control the upper body of the robot, providing joint and cartesian control modes for the arms.
- **ui_interface**: Node to provide a user interface to control the main functionalities of the robot.
## Usage

### Configuration File
The configuration file is located in the `config` folder. You can modify the parameters according to your needs. It's important to set up all the correct information for your robot.

### Instructions
Once you have the docker image running, you can run the following command to start the unitree node:

```bash
colcon build
```

Then, source the workspace:

```bash
source install/setup.bash
```

You can launch the bringup robot with the following command:

```bash
ros2 launch g1pilot bringup_launcher.launch.py
```

### Simulation (MuJoCo) vs. real robot

The sim/real switch is driven by a single environment variable, `G1_SIM_MODE`
(see `g1pilot/utils/common.py`):

| Mode | `G1_SIM_MODE` | Unitree DDS domain | `ROS_DOMAIN_ID` | Interface | Entry point |
|------|---------------|--------------------|-----------------|-----------|-------------|
| Simulation | `true`  | 1 | 0 | `lo` | `bringup_sim.launch.py` |
| Real robot | `false` | 0 | 1 | `${ROBOT_INTERFACE}` | `bringup_real.launch.py` |
| Real robot (full nav stack) | `false` | 0 | 1 | `${ROBOT_INTERFACE}` | `bringup_launcher.launch.py` |

> The Unitree DDS domain (used by `unitree_sdk2py`) and `ROS_DOMAIN_ID` (the
> rmw/ROS graph) must be **different** values — using the same number makes
> nodes that use both ROS and the Unitree SDK crash with a CycloneDDS
> "create domain error". The real G1 transmits on Unitree domain 0 (fixed by
> firmware), so the pairing is mirrored: sim = ROS 0 / Unitree 1, real =
> ROS 1 / Unitree 0.

Easiest entry point — the interactive start menu. Its **first question is
SIM vs. REAL**; the sim branch asks RViz/hands/rebuild, the real branch asks
for the network interface (auto-detected), hand IPs and a typed safety
confirmation:

```bash
./start.sh
# Non-interactive sim (takes defaults / env overrides):
USE_RVIZ=true ./start.sh --yes
# Non-interactive real (requires explicit confirmation):
G1_MODE=real ROBOT_INTERFACE=enp3s0 G1_REAL_CONFIRM=1 ./start.sh --yes
```

The menu controls these env vars (also usable directly):

| Env | Werte | Wirkung |
|-----|-------|---------|
| `G1_MODE` | `sim` (Default) / `real` | Simulation oder echter Roboter. |
| `USE_RVIZ` | sim: `false` / real: `true` (Defaults) | RViz mitstarten (auf real das IK-Marker-Interface). |
| `G1_INSPIRE_HANDS` | `0` / `1` | Inspire-FTP-Haende (sim: MuJoCo-Finger; real: Modbus TCP). |
| `G1_HAND_LEFT_HOST` / `G1_HAND_RIGHT_HOST` / `G1_HAND_PORT` | IPs/Port | Modbus-Ziele der echten Haende (Default `.210`/`.211`:6000). |
| `ROBOT_INTERFACE` | NIC-Name | Physisches Interface zum G1 (real). |
| `G1_MAX_VX` / `G1_MAX_VY` / `G1_MAX_VYAW` | m/s bzw. rad/s | Walk-Limits von Streamdeck/PS4 auf real (Default 0.4/0.3/0.4). |
| `SIM_LOCKSTEP` | `1` (Default) / `0` | Deterministische 50-Hz-Regelrate (nur Sim). |

Recommended (consolidated) Docker entry point:

```bash
# Simulation: starts the MuJoCo G1 sim + g1pilot (robot_state, arms, RViz, teleop)
G1_SIM_MODE=true docker compose --profile sim up

# Real robot (lean: arms + hands + Unitree loco controller)
ROBOT_INTERFACE=<iface> docker compose --profile real up

# Real robot with Livox/MOLA/nav (big image, MID360 required)
ROBOT_INTERFACE=<iface> docker compose --profile real-full up
```

**First time on real hardware? Read `REAL_TESTING.md` (safety checklist +
step-by-step runbook) before starting anything.**

To move the arms: enable the arms and drag the interactive end-effector
markers in RViz (or publish a `PoseStamped` to `/g1pilot/hand_goal/{left,right}`):

```bash
ros2 topic pub -1 /g1pilot/arms/enabled std_msgs/Bool "{data: true}"
```

> Balancing/locomotion: on real hardware via the Unitree onboard high-level
> (`loco_client` — START/START BALANCING/WALK from the Streamdeck, plus PS4);
> in sim via the whole-body policy (`loco_sim`). Both consume the same
> Streamdeck topics, so the UI behaves identically in both modes.

Or you can run each node separately according to your needs.

1.- To run the Livox LiDAR, you can run the following command:

```bash
ros2 launch g1pilot livox_launcher.launch.py
```

2.- To run the mola odometry, you can run the following command:

```bash
ros2 launch g1pilot mola_launcher.launch.py
```

3.- To run the navigation stack and enable the locomotion of the robot, you can run the following command:

```bash
ros2 launch g1pilot navigation_launcher.launch.py
```

4.- To run the manipulation stack, you can run the following command:

```bash
ros2 launch g1pilot manipulation_launcher.launch.py
```

5.- To run the teleoperation stack, you can run the following command:

```bash
ros2 launch g1pilot teleoperation_launcher.launch.py
```

6.- You can run the depth camera on the robot with the following command:
```bash
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true
```

## Entrypoints
TODO

## Contributing
We welcome contributions to **G1Pilot**! If you have suggestions, improvements, or bug fixes, please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Make your changes and commit them with clear messages.
4. Submit a pull request detailing your changes.


## Maintainer
This package is maintained by:

**Clemente Donoso**  
Email: [clemente.donoso@inria.fr](mailto:clemente.donoso@inria.fr)
GitHub: [CDonosoK](https://github.com/CDonosoK)  

## License
BSD‑3‑Clause. See [LICENSE](LICENSE) for details.