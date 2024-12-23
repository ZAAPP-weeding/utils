ZAAPP
=====

This is the public repo of MRSD Team F24 – ZAAPP. It encompasses a weeding robot.


installation
=============

Installation can be done via the `install.bash` script. This project requires Ubuntu 22.04.

execution
===========

To run the ROS2 nodes, navigate to the workspace in a terminal window and then run `bash zaapp/scripts/launch.bash`

The project's ROS2 nodes are typically organized into a structured tmux session for efficient monitoring and control of the robot's operations.


---

### Nodes Launched by `launch.bash`

### **Husky Nodes**
The `husky` window contains four panes, each launching a third party ROS 2 node:
1. **Base Node**:
   - **Command**: `ros2 launch husky_base base_launch.py`
   - **Description**: Launches the base configuration for the Husky robot.

2. **Teleoperation Node**:
   - **Command**: `ros2 launch husky_control teleop_launch.py`
   - **Description**: Launches the teleoperation control for the Husky robot.

3. **Driver Node**:
   - **Command**: `ros2 launch robot_bringup drivers_hack.launch.py`
   - **Description**: Brings up the LiDAR's drivers.

4. **Localization Node**:
   - **Command**: `ros2 launch robot_bringup localization.launch.py`
   - **Description**: Launches FASTLIO localization for the robot.

---

### **Core Nodes**
The `core` window also contains four panes, containing custom drivers:
1. **Weed Detection Node**:
   - **Command**: `ros2 run downward_facing_camera weed_detection_node`
   - **Description**: Processes data from the downward-facing camera for weed detection.

2. **Weed Map Node**:
   - **Command**: `ros2 run forward_facing_camera weed_map_node`
   - **Description**: Processes data from the forward-facing camera to generate a weed map.

3. **Manipulator Node**:
   - **Command**: `ros2 run manipulator_ros manipulator_node`
   - **Description**: Controls the robot's 2DOF manipulator + laser.

4. **Odom2World Node**:
   - **Command**: `ros2 run odom2world odom2world`
   - **Description**: Converts odometry data to a predefined world frame.

---

### **Control Nodes**
The `control` window contains two panes for control and traj generation:
1. **Trajectory Generator Node**:
   - **Command**: `ros2 run behavior trajectory_generator_node`
   - **Description**: Generates trajectories for the robot to follow.

2. **Controller Node**:
   - **Command**: `ros2 run behavior controller_node`
   - **Description**: Controls the robot's movements based on generated trajectories.

---

### **Behavior Nodes**
The `behavior` window has two panes for higher-level behaviors:
1. **Visual Servoing Node**:
   - **Command**: `ros2 run visual_servoing visual_servoing_server`
   - **Description**: Implements visual servoing to guide the robot using visual inputs.

2. **Aladeen Node**:
   - **Command**: `ros2 run behavior aladeen_node --ros-args -p sentience:=0`
   - **Description**: Highest level behavior planning.

---

The final `debug` window does not launch any nodes and is for collaborative debugging.
