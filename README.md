# 3DOF Robot Arm (ROS 2 + MoveIt 2)

This repository contains a full ROS 2 workspace implementing a custom **3-DOF robotic arm**, including:

- A URDF/XACRO model with STL meshes  
- A custom **analytic inverse kinematics plugin**  
- A MoveIt 2 configuration package  
- RViz visualization and motion planning launch files  

This workspace is organized for **ROS 2 Humble** on Linux (WSL2 or Ubuntu).

---

## Workspace Structure

```text
ros2_ws/
├── src/
│ ├── my_arm_description/ # URDF, meshes, RViz display launch
│ ├── my_arm_kinematics/ # Custom IK plugin (analytic solver)
│ └── my_arm_moveit_config/ # MoveIt 2 configuration package
├── build/ # CMake build artifacts (ignored)
├── install/ # Installed package files (ignored)
└── log/ # Build/run logs (ignored)

```

Only the `src/` folder contains tracked source code.

---

## Robot Description

The robot is modeled as a simple 3-DOF manipulator:

1. **Joint 1 — Base yaw**  
2. **Joint 2 — Shoulder pitch**  
3. **Joint 3 — Elbow pitch**

Meshes for all links are located in: `src/my_arm_description/meshes/`

The primary URDF/XACRO file is: `src/my_arm_description/urdf/my_arm.urdf.xacro`

---

## Custom Kinematics Plugin FK/IK

This project includes a custom analytic IK solver implemented as a MoveIt 2 kinematics plugin.

Plugin source: `src/my_arm_kinematics/src/my_arm_kinematics_plugin.cpp`
Plugin export description: `src/my_arm_kinematics/my_arm_kinematics_plugin_description.xml`
MoveIt is configured to load this solver via: `src/my_arm_moveit_config/config/kinematics.yaml`

The solver computes closed-form IK using the geometric relationships between link lengths:

- `L1 = 135 mm`
- `L2 = 147 mm`
- `hs = 96.122711 mm` (shoulder height offset)

---

## Building the Workspace

From inside the workspace (this repository’s root):

```bash
cd ros2_ws
colcon build
source install/setup.bash

```

## Launching RViz with MoveIt

To visualize the robot with MoveIt:

```bash
ros2 launch my_arm_moveit_config demo.launch.py

```

To view the raw URDF in RViz:

```bash
ros2 launch my_arm_description display.launch.py

```

## Controllers

This project currently uses simulated controllers configured via: src/my_arm_moveit_config/config/ros2_controllers.yaml

You can extend this file to match real hardware.

---

## Requirements

* ROS 2 Humble
* MoveIt 2
* colcon build tools
* RViz

---

## Future Work

* Refine wrist orientation constraints in the analytic IK
* Add Gazebo simulation support
* Add hardware controllers for a physical robot
* Add grasping and Cartesian control demos
