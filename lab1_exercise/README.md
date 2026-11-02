# Lab 1 Exercise: Program diffdrive_bot

**Course:** Introduction to Robotics and Intelligent Systems  
**Program:** BSc in Software Engineering  
**Academic Year:** 2026  
**Environment:** ROS 2 Jazzy, Gazebo Harmonic, Ubuntu 24.04 LTS (WSL2)

---

## 1. Project Purpose

This project completes the Lab 1 programming assignment for **Introduction to Robotics and Intelligent Systems**. It extends the differential-drive robot (`diffdrive_bot`) simulated in **Gazebo Harmonic** with two additional sensors (a rear-facing depth camera and an IMU), bridges sensor streams to **ROS 2 Jazzy**, and implements custom ROS 2 packages demonstrating fundamental robotic middleware concepts:
- **Topics & Custom Messages:** Reactive obstacle avoidance and robot state broadcasting.
- **Parameters:** Dynamic runtime speed tuning.
- **Services:** Enabling/disabling motion and runtime speed constraints.
- **Actions:** In-place rotation tracking with odometry yaw unwrap accumulation.
- **Action Clients & Launch Orchestration:** Multi-action choreography driving the robot in a precise square trajectory.

---

## 2. Environment & Compatibility (ROS 2 Jazzy & Gazebo Harmonic)

The original lab repository was written for ROS 2 Humble (Ubuntu 22.04). In this workspace, the environment is **ROS 2 Jazzy** on **Ubuntu 24.04 LTS (WSL2)** paired with **Gazebo Harmonic**.

### Compatibility Adjustments Made:
1. **Bridge Configuration (`bridge.yaml`):**
   - Corrected message type naming for ROS 2 Jazzy (`sensor_msgs/msg/Imu` instead of deprecated or mis-cased types).
   - Bridged Gazebo Harmonic depth camera images (`gz.msgs.Image`), packed point clouds (`gz.msgs.PointCloudPacked`), camera info (`gz.msgs.CameraInfo`), and IMU (`gz.msgs.IMU`).
2. **Gazebo System Plugins (`obstacles.sdf`):**
   - Integrated `gz-sim-imu-system` (`gz::sim::systems::Imu`) alongside `gz-sim-sensors-system` (OGRE2 rendering engine) to ensure IMU sensor measurements are simulated and published in Gazebo Harmonic.
3. **Execution Model & Python 3.12:**
   - Designed all Python nodes using `MultiThreadedExecutor` and `ReentrantCallbackGroup` where appropriate to ensure concurrent sensor stream handling (especially odometry and lidar callbacks during action loops).

---

## 3. Project Structure

The project lives in `~/ros2_ws/src/lab1_exercise` and consists of four independent ROS 2 packages:

```
lab1_exercise/
├── diffdrive_bot/              # Extended robot description, simulation world & launch files
│   ├── config/
│   │   └── bridge.yaml         # ROS-Gazebo bridge mappings (including depth camera & IMU)
│   ├── launch/
│   │   ├── control.launch.py   # Starts Gazebo, nodes, and bridge
│   │   ├── display.launch.py   # RViz URDF inspection
│   │   └── gazebo.launch.py    # Gazebo simulation launch
│   ├── rviz/
│   │   └── diffdrive_bot.rviz  # Configured with Camera, Depth Camera, & PointCloud2 displays
│   ├── scripts/                # Lidar detector, baseline controller, distance action server
│   ├── urdf/
│   │   └── diffdrive_bot.urdf.xacro # URDF with rear depth camera & central IMU
│   ├── worlds/
│   │   └── obstacles.sdf       # SDF world with IMU system plugin
│   ├── CMakeLists.txt
│   └── package.xml
│
├── lab_interfaces/             # Original assignment interfaces
│   ├── action/DriveDistance.action
│   ├── msg/ObstacleInfo.msg
│   ├── srv/SetSpeed.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
├── my_robot_interfaces/        # Custom message, service, and action definitions
│   ├── action/Rotate.action    # In-place rotation action (goal: angle, result/feedback: turned)
│   ├── msg/RobotStatus.msg     # Custom status message (state: forward/turning/stopped, obstacle_distance)
│   ├── srv/SetTurnSpeed.srv    # Custom service to validate and set turn speed
│   ├── CMakeLists.txt
│   └── package.xml
│
├── my_robot_nodes/             # Custom ROS 2 Python nodes and square launch file
│   ├── launch/
│   │   └── square.launch.py    # Master launch orchestrating full simulation and square trajectory
│   ├── my_robot_nodes/
│   │   ├── __init__.py
│   │   ├── turn_away.py        # Reactive obstacle avoidance node with services and status publisher
│   │   ├── rotate_server.py    # Action server tracking rotation via /odom yaw integration
│   │   └── square_driver.py    # Action client driving 4 sides of a square
│   ├── resource/my_robot_nodes
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
│
├── screenshots/                # Verification screenshots
│   ├── rviz_depth_camera.png   # RViz view with depth camera image & point cloud
│   └── rqt_graph_square.png    # rqt_graph computation graph during square execution
├── .gitignore
└── README.md
```

To prevent package name collisions with the original lab clone, `~/ros2_ws/src/intro_robotics_intelligent_systems_labs/lab_01/COLCON_IGNORE` is present so `colcon` builds exclusively from `lab1_exercise`.

---

## 4. Build Instructions

Open your WSL2 terminal (Ubuntu 24.04):

```bash
# 1. Source ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# 2. Navigate to workspace root
cd ~/ros2_ws

# 3. Build all packages with symlink install
colcon build --symlink-install

# 4. Source the workspace overlay
source install/setup.bash
```

---

## 5. Running and Testing Each Exercise Stage

> [!IMPORTANT]
> Always source `/opt/ros/jazzy/setup.bash` and `~/ros2_ws/install/setup.bash` in **every new terminal window**.

### Stage 1: Verify Depth Camera and IMU Sensors

1. **Inspect URDF Model in RViz:**
   ```bash
   ros2 launch diffdrive_bot display.launch.py
   ```
   *Expected:* RViz opens showing the robot. Verify `depth_camera_link` exists on the rear face (facing backward) and `imu_link` is at the centre of the chassis.

2. **Verify TF Transform:**
   ```bash
   ros2 run tf2_ros tf2_echo base_link depth_camera_link
   ```
   *Expected:* Translation is approximately `[-0.215, 0.0, 0.0]` m and rotation around the z-axis is approximately `3.142` rad ($\pi$).

3. **Launch Gazebo Simulation:**
   ```bash
   ros2 launch diffdrive_bot gazebo.launch.py
   ```
   *Expected:* Gazebo opens with the robot facing the red box with the rear wall behind it. RViz opens displaying:
   - `/camera/image`: front view of the red box.
   - `/depth_camera/image`: depth image of the wall behind the robot.
   - `/depth_camera/points`: 3D point cloud of the wall.

4. **Verify Sensor Topics & Frequencies:**
   ```bash
   # Check IMU rate (~50 Hz)
   ros2 topic hz /imu

   # Check IMU acceleration (~9.8 m/s^2 on Z axis)
   ros2 topic echo /imu --field linear_acceleration.z

   # Check depth camera info
   ros2 topic echo /depth_camera/camera_info --once
   ```

---

### Stages 2, 3, 4 & 5: Reactive Obstacle Avoidance (`turn_away` Node)

The `turn_away` node implements:
- Obstacle avoidance (turns away when obstacle is closer than stop threshold).
- Dynamic parameter tuning (`forward_speed`, `turn_speed`).
- Custom `/robot_status` publishing (`my_robot_interfaces/msg/RobotStatus`).
- Enable/disable service (`/turn_away_enable`).
- Speed configuration service (`/set_turn_speed`).

#### Execution Steps:
1. **Launch Robot Control Simulation (Terminal 1):**
   ```bash
   ros2 launch diffdrive_bot control.launch.py
   ```
   *(Note: Do not call `/drive_enable` so only `turn_away` controls the robot.)*

2. **Start `turn_away` Node (Terminal 2):**
   ```bash
   ros2 run my_robot_nodes turn_away
   ```
   *Behavior:* The node starts **disabled**. The robot remains stationary.

3. **Echo Robot Status (Terminal 3):**
   ```bash
   ros2 topic echo /robot_status
   ```
   *Expected while disabled:* `state: "stopped"`, `obstacle_distance` reflects distance to obstacle.

4. **Enable the Controller (Stage 5 Service):**
   ```bash
   ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
   ```
   *Expected:* The robot begins driving forward toward the obstacle (`state: "forward"`). When `too_close` becomes true, the robot stops linear motion and turns away on the spot (`state: "turning"`). Once clear, it drives forward again.

5. **Test Dynamic Parameters (Stage 3):**
   ```bash
   ros2 param list /turn_away
   ros2 param set /turn_away forward_speed 0.4
   ros2 param set /turn_away turn_speed 0.8
   ```
   *Expected:* The robot visibly accelerates immediately because parameters are evaluated on every obstacle callback.

6. **Test `/set_turn_speed` Service (Stage 5):**
   ```bash
   # Test valid speed (accepted):
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 1.0}"

   # Test speed exceeding max_turn_speed (rejected):
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 3.0}"

   # Test invalid zero/negative speed (rejected):
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 0.0}"
   ```

7. **Disable the Controller:**
   ```bash
   ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
   ```
   *Expected:* The robot immediately stops, and `/robot_status` reverts to `state: "stopped"`.

---

### Stage 6: In-Place Rotation Action Server (`rotate_server`)

`rotate_server` provides the `/rotate` action (`my_robot_interfaces/action/Rotate`), unwrapping yaw angles from `/odom` to handle $\pm \pi$ jumps seamlessly.

1. **Launch Simulation (if not already running):**
   ```bash
   ros2 launch diffdrive_bot control.launch.py
   ```

2. **Run the Action Server:**
   ```bash
   ros2 run my_robot_nodes rotate_server
   ```

3. **Send a Rotation Goal (Quarter turn left, ~1.57 rad):**
   ```bash
   ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
   ```
   *Expected Behavior:*
   - Goal accepted.
   - Feedback continuously reports `turned` climbing toward `1.57` rad.
   - The robot turns in place, stops precisely at 90 degrees, and returns `turned: 1.57`.

4. **Verify Heading with TF:**
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```

5. **Test Goal Rejection (Zero Angle):**
   ```bash
   ros2 action send_goal /rotate my_robot_interfaces/action/Rotate "{angle: 0.0}"
   ```
   *Expected:* Goal is rejected immediately.

---

### Stage 7: Square Trajectory Execution (`square_driver` and `square.launch.py`)

The `square_driver` action client sequences four `DriveDistance` and four `Rotate` goals to drive the robot around a closed square trajectory.

#### Run the Complete System with One Command:
```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```

*Expected Behavior:*
1. Gazebo Harmonic, the ROS-Gazebo bridge, RViz, and diffdrive nodes start automatically.
2. `rotate_server` and `square_driver` initialize.
3. For each of the 4 sides:
   - Robot drives forward `0.8` meters (`/drive_distance`).
   - Robot turns left 90 degrees in place (`/rotate`).
4. Upon completing side 4, the robot returns close to its initial starting pose and halts safely.

To run with the default `0.5` m side length:
```bash
ros2 launch my_robot_nodes square.launch.py
```

---

## 6. Required Screenshots

The assignment requires two verification screenshots saved in `lab1_exercise/screenshots/`:

1. **`screenshots/rviz_depth_camera.png`:**
   - Launch: `ros2 launch diffdrive_bot gazebo.launch.py`
   - In RViz, verify the 3D robot, the depth camera image panel (`/depth_camera/image`), and the point cloud (`/depth_camera/points`).
   - Capture window and save to `lab1_exercise/screenshots/rviz_depth_camera.png`.

2. **`screenshots/rqt_graph_square.png`:**
   - Launch: `ros2 launch my_robot_nodes square.launch.py side_length:=0.8`
   - In another terminal, run: `rqt_graph`
   - Set filter to Nodes/Topics (all) to capture active nodes (`square_driver`, `rotate_server`, `drive_distance_server`, `parameter_bridge`).
   - Save to `lab1_exercise/screenshots/rqt_graph_square.png`.

---

## 7. Git Submission Instructions

The project folder `lab1_exercise` is self-contained and ready to be maintained as its own Git repository:

```bash
cd ~/ros2_ws/src/lab1_exercise

# Initialize git repository (if not already done)
git init
git checkout -b main

# Stage all files
git add .

# Create initial commit
git commit -m "feat: complete Lab 1 diffdrive_bot exercise across all 7 stages"
```

To push to your remote GitHub repository once confirmed:
```bash
git remote add origin <YOUR_GITHUB_REPO_URL>
git branch -M main
git push -u origin main
```
