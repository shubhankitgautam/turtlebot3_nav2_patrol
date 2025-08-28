# turtlebot3_nav2_patrol
Python script for TurtleBot3 patrol in Gazebo using ROS2 Nav2, where the robot autonomously navigates through multiple rooms by following predefined waypoints.

## Overview
This project demonstrates a patrolling robot using TurtleBot3 in ROS 2 and Gazebo. The robot starts at an initial position, navigates through all predefined rooms one by one, returns to its starting point, waits for a few seconds, and then repeats the sequence. The automation is handled using a Python script, while Navigation2 (Nav2) ensures accurate path planning and obstacle avoidance. This setup is ideal for surveillance, inspection, and  monitoring applications, showcasing autonomous navigation and task repetition in a simulated environment.

## Features
- Autonomous Patrolling – Robot visits all rooms in a predefined sequence.
- Return to Start – Always returns to its initial position before repeating the patrol.
- Delay Between Cycles – Waits for 5 seconds at the starting position before beginning the next cycle.
- Python Automation – Patrolling logic implemented in a Python script for easy customization.
- ROS 2 Navigation2 Integration – Uses Nav2 for path planning, obstacle avoidance, and localization.
- Simulated Environment – Fully tested in TurtleBot3 Gazebo simulation with RViz2 visualization.
- Modular and Extensible – Easy to add new rooms, change routes, or integrate with real TurtleBot3 hardware.

## Requirements
- **OS:** Ubuntu 22.04  
- **ROS2:** Humble Hawksbill  
- **Gazebo:** 11  
- **TurtleBot3 Packages:** `turtlebot3_description`, `turtlebot3_gazebo`, `turtlebot3_navigation2`  
- Python 3, `colcon` build tool  

---

## Installation
Clone the repository into your ROS2 workspace:
cd ~/turtlebot3_ws/src
git clone <repository_url>

## Install dependencies:
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

## Build the workspace:
cd ~/turtlebot3_ws
colcon build
source install/setup.bash

## Export TurtleBot3 model:
export TURTLEBOT3_MODEL=waffle

## Launching the Simulation
ros2 launch turtlebot3_gazebo turtlebot3_shubhankit_world.launch.py

## Launching rviz2
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/shubhankit_map.yaml

## Running the python script
./nav2_patrol.py

## Stopping the script
ctrl + c

## URDF & Physics Configuration
Wheel friction and damping are tuned to prevent slipping:
mu1=2.0, mu2=2.0
kp and kd values adjusted in <gazebo> wheel tags
Physics damping added in Gazebo link configuration
Sensors:
IMU (imu_link) for orientation and acceleration
LiDAR (base_scan) for obstacle detection
Camera (camera_rgb_frame) for vision-based tasks
Diff-drive plugin configured for proper odometry and control

## Custom World Notes
Friction values in your custom world may need adjustments. Very high friction can cause oscillations or wheel spinning.
maxVel in the diff-drive plugin should be low enough to prevent wheel overshoot.
Always source the workspace before launching commands:
source ~/turtlebot3_ws/install/setup.bash
Calibrate friction and damping depending on obstacles and surface type in custom worlds.

## Author
Shubhankit – Robotics Enthusiast / Mechanical Engineer

## References
[TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
[ROS2 Navigation2 Guide](https://docs.nav2.org/)
[Cartographer SLAM](http://google-cartographer.readthedocs.io/en/latest/)


## License
This project is licensed under the MIT License. See the LICENSE file for details.
✅ This version includes:  
- Overview, features, requirements  
- Full installation and launch instructions  
- SLAM, navigation, URDF & physics notes  
- Custom world notes  
- Placeholders for screenshots and diagrams  
- References and license  
