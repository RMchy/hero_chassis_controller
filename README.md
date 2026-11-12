# hero_chassis_controller

## Overview

Mecanum-wheel chassis controller for the RoboMaster **hero** robot, written for the
Guangdong University of Technology **DynamicX** team 27-season reserve-member final
assignment.

It is a `ros_control` controller plugin that:

- subscribes `/cmd_vel` (`geometry_msgs/Twist`),
- converts the chassis velocity into four wheel target speeds with **mecanum inverse kinematics**,
- tracks each wheel speed with its own **`control_toolbox::Pid`** loop and outputs joint **effort**,
- reconstructs the chassis velocity from the measured wheel speeds with **forward kinematics**,
- integrates it into an **odometry** pose, published as `nav_msgs/Odometry` on `/odom`
  together with the tf transform `odom` -> `base_link`,
- optionally re-interprets `/cmd_vel` as a **world-frame** velocity and rotates it into
  `base_link` with tf (assignment 7),
- optionally limits the command acceleration (assignment 8).

**Keywords:** RoboMaster, ROS, ros_control, mecanum, odometry, PID

### License

The source code is released under a BSD 3-Clause license (inherited from the
`rm_template` repository this package was created from).

**Author: `<Your Name>`**<br />
**Affiliation: GDUT DynamicX**<br />
**Maintainer: `<Your Name>`, `<your_email@example.com>`**

Tested under ROS Noetic on Ubuntu 20.04.

## Installation

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) Noetic,
- [rm_description_for_task](https://github.com/Bluefairy-ljy/rm_description_for_task) (hero chassis model),
- `rm_gazebo` (world files; a sub-package of
  [rm_control](https://github.com/Bluefairy-ljy/rm_control) — copy only that folder into your workspace),
- `control_toolbox`, `controller_interface`, `hardware_interface`, `forward_command_controller`,
  `pluginlib`, `tf2_ros`, `tf2_geometry_msgs`, `dynamic_reconfigure`,
- `teleop_twist_keyboard` (for testing / demo).

### Building

    cd ~/catkin_ws/src
    git clone <this repository>
    cd ..
    rosdep install --from-paths . --ignore-src -r -y
    catkin build            # catkin build, NOT catkin_make (assignment rule)
    source devel/setup.bash

## Usage

Run the simulation and the controller together:

    roslaunch hero_chassis_controller run_simulation_and_controller.launch

Load the controller onto an already-running simulation / real robot:

    roslaunch hero_chassis_controller load_controller.launch

Drive it with the keyboard (in a separate terminal):

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

## Config files

* **controllers.yaml** — per-wheel PID gains (`left_front_pid` … `right_back_pid`),
  kinematic parameters (`wheel_radius`, `wheel_track`, `wheel_base`),
  the velocity-mode switch (`velocity_mode`: `base_link` or `odom`) and the
  acceleration limit (`max_acceleration`).

## Launch files

* **run_simulation_and_controller.launch:** spawns the hero chassis in Gazebo,
  loads the YAML, spawns this controller plus `joint_state_controller`, and starts
  the keyboard teleop node. Argument `roller_type` (`simple` for development,
  `realistic` for the final demo recording).
* **load_controller.launch:** same controller loading without starting Gazebo.

## Controller

### hero_chassis_controller/HeroChassisController

#### Subscribed Topics

* **`/cmd_vel`** (`geometry_msgs/Twist`) — chassis velocity command. Interpreted in
  `base_link` or in the world frame depending on `velocity_mode`.

#### Published Topics

* **`/odom`** (`nav_msgs/Odometry`) — integrated odometry pose and twist.
* **`/hero_chassis/debug/velocity`** (`std_msgs/Float64MultiArray`) — eight values:
  four target wheel speeds followed by four measured wheel speeds, for `rqt_plot`.

#### Broadcast tf

* **`odom` -> `base_link`** — from the integrated forward-kinematics odometry.

#### Parameters

* **`wheel_radius`** (double, default 0.07625) — wheel radius in metres.
* **`wheel_track`** (double, default 0.4) — lateral distance between wheel centres.
* **`wheel_base`** (double, default 0.4) — longitudinal distance between wheel centres.
* **`velocity_mode`** (string, default "base_link") — "base_link" or "odom".
* **`max_acceleration`** (double, default 0 = disabled) — command ramp limit in m/s².
* **`<wheel>_pid/{p,i,d,i_clamp_max,i_clamp_min}`** — gains of each wheel's PID.

## Features completed

<!-- Keep this list updated as you finish each requirement (assignment 1 asks for it). -->

- [ ] Mandatory 1: run `rm_description_for_task` master branch
- [ ] Mandatory 2: understand and run `simple_chassis_controller`, CLion debug attached to gzserver
- [ ] Assignment 1: package created from `rm_template`, README/CMakeLists/package.xml filled
- [ ] Assignment 2: clean commit history incl. a resolved merge conflict
- [ ] Assignment 3: code passes `roslint_cpp()` / clang-format
- [ ] Assignment 4: four PIDs, gains in YAML, tuned with rqt_plot + rqt_reconfigure
- [ ] Assignment 5: inverse kinematics, all four directions + diagonal verified
- [ ] Assignment 6: odometry on /odom and tf odom -> base_link
- [ ] Assignment 7: world-frame velocity mode switchable
- [ ] Assignment 8: extra feature (acceleration limiting)

## Bugs & Feature Requests

Report them via the repository's Issue Tracker.

[ROS]: http://www.ros.org
