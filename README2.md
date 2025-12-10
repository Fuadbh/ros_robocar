# ROS Robocar - System Architecture

## Overview

This project implements a complete ROS 2-based autonomous robotic car system designed for SLAM (Simultaneous Localization and Mapping) and autonomous navigation. The system supports simulation in Gazebo and can be controlled via joystick or autonomous navigation stack.

## System Architecture

### High-Level Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo Simulation                        │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │   Robot      │  │    LiDAR     │  │      Camera         │   │
│  │   Model      │  │   (360°)     │  │   (640x480 RGB)     │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬──────────┘   │
└─────────┼──────────────────┼──────────────────────┼─────────────┘
          │                  │                      │
          │                  │                      │
          ▼                  ▼                      ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 Control Layer                           │
│  ┌──────────────────┐         ┌──────────────────────────┐     │
│  │ Diff Drive       │         │  Joint State             │     │
│  │ Controller       │         │  Broadcaster             │     │
│  └────────┬─────────┘         └──────────┬───────────────┘     │
└───────────┼───────────────────────────────┼─────────────────────┘
            │                               │
            │                               │
            ▼                               ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Robot State Publisher                         │
│              (TF Tree: map → odom → base_footprint)              │
└─────────────────────────────────────────────────────────────────┘
            │
            │
┌───────────┼───────────────────────────────────────────────────────┐
│           │         Command Layer                                  │
│           │  ┌──────────────────────────────────────┐            │
│           │  │          twist_mux                   │            │
│           │  │  (Priority-based command multiplexer) │            │
│           │  │  - Navigation (priority: 10)          │            │
│           │  │  - Tracker (priority: 20)             │            │
│           │  │  - Joystick (priority: 100)           │            │
│           │  └────────────┬─────────────────────────┘            │
│           │               │                                       │
│           │    ┌──────────┴──────────┐                          │
│           │    │                     │                          │
│           ▼    ▼                     ▼                          │
│   ┌────────────┐         ┌──────────────────┐                  │
│   │ Navigation │         │   Joystick       │                  │
│   │ Stack      │         │   Teleop         │                  │
│   │ (Nav2)     │         │   (teleop_twist) │                  │
│   └─────┬──────┘         └──────────────────┘                  │
│         │                                                    │
└─────────┼────────────────────────────────────────────────────┘
          │
          │
┌─────────┼─────────────────────────────────────────────────────────┐
│         │         Perception & Localization Layer                  │
│         │  ┌──────────────────┐      ┌─────────────────────┐     │
│         │  │   SLAM Toolbox   │      │   AMCL (Localization)│     │
│         │  │  (Online Async)  │      │   Map Server        │     │
│         │  └────────┬─────────┘      └──────────┬──────────┘     │
│         │           │                           │                │
│         │           ▼                           ▼                │
│         │    ┌──────────────────────────────────────┐            │
│         │    │         Sensor Data                  │            │
│         │    │   - /scan (LaserScan)                │            │
│         │    │   - /odom (Odometry)                 │            │
│         │    │   - /camera/image_raw                │            │
│         │    └──────────────────────────────────────┘            │
└─────────┼─────────────────────────────────────────────────────────┘
          │
          ▼
┌─────────────────────────────────────────────────────────────────┐
│                    Navigation Stack (Nav2)                       │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │  Planner     │  │  Controller  │  │  Recovery Behaviors │   │
│  │  Server      │  │  Server      │  │  Server             │   │
│  └──────┬───────┘  └──────┬───────┘  └──────────┬──────────┘   │
│         │                 │                      │              │
│         └─────────────────┼──────────────────────┘              │
│                           │                                     │
│                  ┌────────▼────────┐                           │
│                  │  BT Navigator   │                           │
│                  │  (Behavior Tree)│                           │
│                  └─────────────────┘                           │
└─────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Robot Physical Model

**Location**: `description/robot.urdf.xacro`

The robot is a differential drive platform with the following components:

- **Chassis**: 
  - Dimensions: 0.335m × 0.265m × 0.138m
  - Mass: 1.0 kg
  - Material: White

- **Drive Wheels** (2×):
  - Radius: 0.033 m
  - Thickness: 0.026 m
  - Mass: 0.05 kg each
  - Wheelbase: 0.297 m (separation between wheels)
  - Position: ±0.1485 m from center

- **Caster Wheel**:
  - Radius: 0.01 m
  - Provides stability (passive wheel)

### 2. Sensors

#### LiDAR Sensor
**Location**: `description/lidar.xacro`

- **Type**: 360° Laser Range Finder
- **Specifications**:
  - Frame: `laser_frame`
  - Update Rate: 10 Hz
  - Samples: 360 per scan
  - Range: 0.3 m to 12 m
  - Angular Range: ±180° (full rotation)
- **Topic**: `/scan` (sensor_msgs/LaserScan)
- **Position**: Mounted on chassis at (0.122, 0, 0.212) m

#### Camera
**Location**: `description/camera.xacro`

- **Type**: RGB Camera
- **Specifications**:
  - Resolution: 640 × 480 pixels
  - Format: R8G8B8
  - Horizontal FOV: 1.089 radians (~62.4°)
  - Update Rate: 10 Hz
  - Frame: `camera_link_optical`
- **Topic**: `/camera/image_raw` (sensor_msgs/Image)
- **Position**: Mounted on chassis at (0.276, 0, 0.181) m with 0.18 rad pitch

### 3. Control System

**Location**: `description/ros2_control.xacro`, `config/my_controllers.yaml`

#### ROS 2 Control Architecture

- **Hardware Interface**: Gazebo ROS 2 Control Plugin
- **Controller Manager**: Update rate 30 Hz

**Controllers**:

1. **Differential Drive Controller** (`diff_cont`)
   - Type: `diff_drive_controller/DiffDriveController`
   - Publish Rate: 50 Hz
   - Base Frame: `base_link`
   - Wheel Separation: 0.297 m
   - Wheel Radius: 0.033 m
   - Command Interface: Velocity control
   - Velocity Limits: ±10 rad/s per wheel

2. **Joint State Broadcaster** (`joint_broad`)
   - Type: `joint_state_broadcaster/JointStateBroadcaster`
   - Publishes joint states for both drive wheels

**Control Flow**:
```
/cmd_vel (geometry_msgs/Twist) 
  → Diff Drive Controller 
    → Left/Right Wheel Velocity Commands
      → Gazebo Physics Simulation
```

### 4. Command Multiplexing

**Location**: `config/twist_mux.yaml`

The system uses `twist_mux` to handle multiple velocity command sources with priority:

- **Navigation** (Priority: 10, Timeout: 0.5s)
  - Topic: `/cmd_vel`
  - Used by Nav2 navigation stack

- **Tracker** (Priority: 20, Timeout: 0.5s)
  - Topic: `/cmd_vel_tracker`
  - Reserved for future object tracking functionality

- **Joystick** (Priority: 100, Timeout: 0.5s)
  - Topic: `/cmd_vel_joy`
  - Highest priority for manual override
  - From joystick teleoperation

The multiplexer outputs to `/diff_cont/cmd_vel_unstamped` which is consumed by the differential drive controller.

### 5. Teleoperation

**Location**: `launch/joystick.launch.py`, `config/joystick.yaml`

- **Package**: `joy` + `teleop_twist_joy`
- **Controls**:
  - Left Stick (axis 0): Left-Right (steering)
  - Left Stick (axis 1): Up-Down (forward/backward)
  - Left Bumper (button 4): Additional control
  - Right Bumper (button 5): Additional control
- **Output Topic**: `/cmd_vel_joy`

### 6. SLAM (Simultaneous Localization and Mapping)

**Location**: `launch/online_async_launch.py`, `config/mapper_params_online_async.yaml`

- **Package**: `slam_toolbox`
- **Mode**: Online Asynchronous SLAM
- **Node**: `async_slam_toolbox_node`
- **Inputs**:
  - `/scan` (LaserScan from LiDAR)
  - `/odom` (Odometry)
- **Outputs**:
  - `/map` (OccupancyGrid)
  - `/map_metadata` (MapMetaData)
  - Transform: `map` → `odom`

**Features**:
- Real-time map building while navigating
- Asynchronous processing for performance
- Loop closure detection
- Map persistence capabilities

### 7. Localization

**Location**: `launch/localization_launch.py`, `config/nav2_params.yaml`

**Components**:

1. **AMCL** (Adaptive Monte Carlo Localization)
   - Particle filter-based localization
   - Max particles: 2000
   - Min particles: 500
   - Uses LiDAR scans for pose estimation
   - Frame: `base_footprint` → `map` (via `odom`)

2. **Map Server**
   - Loads pre-built maps (YAML format)
   - Publishes `/map` topic
   - Required for AMCL initialization

**TF Tree**:
```
map → odom → base_footprint → base_link → chassis → [sensors]
```

### 8. Navigation Stack (Nav2)

**Location**: `launch/navigation_launch.py`, `config/nav2_params.yaml`

**Components**:

1. **Controller Server** (`nav2_controller`)
   - Follows the path provided by planner
   - Produces velocity commands (`/cmd_vel`)
   - Uses Dynamic Window Approach (DWA) or similar algorithms

2. **Planner Server** (`nav2_planner`)
   - Plans global path from start to goal
   - Uses A*, Theta*, or NavFn algorithms
   - Considers static obstacles from map

3. **Recovery Behaviors Server** (`nav2_recoveries`)
   - Handles robot recovery from failed states
   - Behaviors: Spin, Backup, Wait

4. **Behavior Tree Navigator** (`nav2_bt_navigator`)
   - Orchestrates navigation using behavior trees
   - Default BT: `navigate_w_replanning_and_recovery.xml`
   - Manages state machine for navigation tasks

5. **Waypoint Follower** (`nav2_waypoint_follower`)
   - Follows sequence of waypoints
   - Useful for patrolling or multi-goal navigation

6. **Lifecycle Manager** (`nav2_lifecycle_manager`)
   - Manages node lifecycle states (Unconfigured → Inactive → Active)
   - Coordinates startup/shutdown of navigation nodes

### 9. Simulation Environment

**Location**: `launch/launch_sim.launch.py`, `config/gazebo_params.yaml`

- **Simulator**: Gazebo Classic
- **Physics Engine**: ODE (default)
- **World Files**: Located in `worlds/` directory
  - `empty.world`: Empty testing environment
  - `obstacles.world`: Environment with obstacles for testing

**Launch Sequence**:
1. Robot State Publisher (publishes TF from URDF)
2. Joystick node (optional manual control)
3. Gazebo simulation
4. Entity spawner (spawns robot in Gazebo)
5. Controller spawners (starts ROS 2 controllers)

### 10. Visualization

**RViz2 Configurations**:

- `main.rviz`: Main visualization for navigation and mapping
- `drive_bot.rviz`: Configuration for robot driving
- `view_bot.rviz`: Robot visualization

**Typical Visualizations**:
- Robot model (URDF)
- LiDAR scans (`/scan`)
- Camera feed (`/camera/image_raw`)
- Map (`/map`)
- Planned path
- Costmaps (global and local)
- TF frames

## Data Flow

### Sensor → SLAM Flow
```
LiDAR (/scan) → slam_toolbox → /map → Nav2 Stack
```

### Navigation Command Flow
```
Goal Pose → Nav2 Planner → Path → Nav2 Controller → /cmd_vel → 
  twist_mux → /diff_cont/cmd_vel_unstamped → Diff Drive Controller → 
    Wheel Velocities → Gazebo Physics
```

### Localization Flow
```
LiDAR (/scan) + Map → AMCL → Pose Estimate → TF (map → base_footprint)
Odometry → TF (odom → base_footprint)
```

## Directory Structure

```
ros_robocar/
├── config/              # Configuration files
│   ├── nav2_params.yaml         # Nav2 stack parameters
│   ├── mapper_params_online_async.yaml  # SLAM parameters
│   ├── joystick.yaml            # Joystick mapping
│   ├── twist_mux.yaml           # Command multiplexer config
│   ├── my_controllers.yaml      # ROS 2 control config
│   └── *.rviz                   # RViz visualization configs
│
├── description/         # Robot description (URDF/XACRO)
│   ├── robot.urdf.xacro         # Main robot description
│   ├── robot_core.xacro         # Chassis and wheels
│   ├── lidar.xacro              # LiDAR sensor
│   ├── camera.xacro             # Camera sensor
│   ├── ros2_control.xacro       # Control interface
│   └── gazebo_control.xacro     # Gazebo control (legacy)
│
├── launch/             # Launch files
│   ├── launch_sim.launch.py     # Main simulation launcher
│   ├── rsp.launch.py            # Robot state publisher
│   ├── joystick.launch.py       # Joystick control
│   ├── online_async_launch.py   # SLAM launcher
│   ├── localization_launch.py   # AMCL localization
│   └── navigation_launch.py     # Nav2 navigation stack
│
└── worlds/             # Gazebo world files
    ├── empty.world              # Empty test environment
    └── obstacles.world          # Environment with obstacles
```

## Key ROS 2 Topics

### Sensor Topics
- `/scan` (sensor_msgs/LaserScan): LiDAR data
- `/camera/image_raw` (sensor_msgs/Image): Camera feed
- `/odom` (nav_msgs/Odometry): Odometry data

### Command Topics
- `/cmd_vel` (geometry_msgs/Twist): Navigation commands
- `/cmd_vel_joy` (geometry_msgs/Twist): Joystick commands
- `/diff_cont/cmd_vel_unstamped` (geometry_msgs/Twist): Controller input

### Navigation Topics
- `/map` (nav_msgs/OccupancyGrid): Map data
- `/goal_pose` (geometry_msgs/PoseStamped): Navigation goal
- `/plan` (nav_msgs/Path): Planned path
- `/local_costmap/costmap` (nav_msgs/OccupancyGrid): Local costmap
- `/global_costmap/costmap` (nav_msgs/OccupancyGrid): Global costmap

### State Topics
- `/joint_states` (sensor_msgs/JointState): Joint positions/velocities
- `/tf` (tf2_msgs/TFMessage): Transform tree
- `/tf_static` (tf2_msgs/TFMessage): Static transforms

## Typical Workflow

### 1. Simulation Startup
```bash
ros2 launch ros_robocar launch_sim.launch.py world:=./src/ros_robocar/worlds/obstacles.world
```

### 2. SLAM Mapping
```bash
ros2 launch ros_robocar online_async_launch.py \
  params_file:=./src/ros_robocar/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

### 3. Navigation (with existing map)
```bash
# Start localization
ros2 launch ros_robocar localization_launch.py \
  map:=./my_map_save.yaml \
  use_sim_time:=true

# Start navigation
ros2 launch ros_robocar navigation_launch.py \
  use_sim_time:=true \
  map_subscribe_transient_local:=true
```

### 4. Visualization
```bash
rviz2 -d src/ros_robocar/config/main.rviz
```

## Dependencies

### ROS 2 Packages
- `nav2_bringup`: Navigation stack
- `slam_toolbox`: SLAM implementation
- `gazebo_ros`: Gazebo integration
- `robot_state_publisher`: TF tree publication
- `joy` + `teleop_twist_joy`: Joystick control
- `controller_manager`: ROS 2 control
- `diff_drive_controller`: Differential drive control
- `joint_state_broadcaster`: Joint state publishing
- `twist_mux`: Command multiplexing

### ROS 2 Distribution
- Tested with ROS 2 Foxy

## Design Patterns

1. **Modular Architecture**: Each component (SLAM, localization, navigation) can run independently
2. **Priority-based Control**: twist_mux ensures safe manual override
3. **Lifecycle Management**: Nav2 uses lifecycle nodes for robust startup/shutdown
4. **Parameter-based Configuration**: All tuning via YAML files
5. **XACRO Macros**: Reusable robot description components

## Future Enhancements

- Object detection and tracking using camera
- Multi-robot coordination
- Dynamic obstacle avoidance beyond static map
- Advanced sensor fusion (IMU, camera, LiDAR)
- Real-world hardware deployment
- Performance monitoring and diagnostics

