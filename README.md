# SLAM + A* Gazebo Simulation Project

This project is a ROS 2 Jazzy + Gazebo simulation for a mobile robot equipped with a 2D lidar.  
The current goal is to build a simulated environment, run SLAM in RViz, and later use the generated map for A* path planning.

## Current status

Currently working:

- Gazebo simulation with a custom SDF world
- Jetbot robot model in Gazebo
- 2D lidar topic bridged from Gazebo to ROS 2
- Odometry topic bridged from Gazebo to ROS 2
- Velocity control through `/cmd_vel`
- RViz visualization of odometry and lidar
- Manual TF publishing:
  - `jetbot/odom -> jetbot/chassis`
  - `jetbot/chassis -> jetbot/lidar/gpu_lidar`
- Initial SLAM Toolbox setup
- Map visualization in RViz
- saved SLAM map loaded from file,
- A* path planning on the saved map,
- path visualization in RViz through `/astar_path`,
- robot velocity control through `/cmd_vel`,
- debug image generation for the planned path.


Still in progress:

- Connecting the saved map to the A* planner
- Driving the robot automatically to a clicked point in RViz

## Requirements

Tested with:

- Ubuntu 24.04 / WSL2
- ROS 2 Jazzy
- Gazebo Sim / gz
- RViz2
- slam_toolbox
- ros_gz_bridge
- teleop_twist_keyboard

Install required packages:


sudo apt update
sudo apt install -y ros-jazzy-ros-gz
sudo apt install -y ros-jazzy-slam-toolbox
sudo apt install -y ros-jazzy-teleop-twist-keyboard
sudo apt install -y ros-jazzy-tf2-tools
sudo apt install -y ros-jazzy-nav2-map-server
sudo apt install -y python3-opencv python3-numpy


How to run :
In every terminal :
- source /opt/ros/jazzy/setup.bash

Terminal 1 :
 
cd ~/slam_astar_project/sdf
gz sim jetbot_world.sdf --render-engine ogre

Terminal 2 - gazebo to ros2 bridge:

ros2 run ros_gz_bridge parameter_bridge \
/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock \
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
/model/jetbot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry

Terminal 3 - publish odometry:

python3 ~/slam_astar_project/src/odom_tf_pub.py

Terminal 4 - publish static lidar TF :

ros2 run tf2_ros static_transform_publisher \
--x 0.0 --y 0.0 --z 1.0 \
--roll 0.0 --pitch 0.0 --yaw 0.0 \
--frame-id jetbot/chassis \
--child-frame-id jetbot/lidar/gpu_lidar

Terminal 5 - Load the saved map :
ros2 run nav2_map_server map_server \
--ros-args \
-p yaml_filename:=/home/$USER/slam_astar_project/maps/slam_map.yaml \
-p use_sim_time:=true


Terminal 6 - Activate the map server : 
- ros2 lifecycle set /map_server configure
- ros2 lifecycle set /map_server activate
- ros2 lifecycle get /map_server

Terminal 7 - publish static map to odom TF
ros2 run tf2_ros static_transform_publisher \
--x 0.0 --y 0.0 --z 0.0 \
--roll 0.0 --pitch 0.0 --yaw 0.0 \
--frame-id map \
--child-frame-id jetbot/odom


Terminal 8 - start RVIZ :
rviz2 -d ~/slam_astar_project/rviz/slam_astar.rviz --ros-args -p use_sim_time:=true


Terminal 9 - start the A* node :
python3 ~/slam_astar_project/src/astar_drive.py


And select a goal point in RVIZ. The A* will :
- read the clicked point
- compute a path on the saved map
- publish the path to /astar_path
- save results/astar_plan.png
- send commands to /cmd_vel

