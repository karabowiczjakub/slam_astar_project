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

Still in progress:

- Improving SLAM map quality
- Saving a clean map
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


Terminal 5 - start SLAM toolbox :

ros2 launch slam_toolbox online_sync_launch.py \
slam_params_file:=/home/$USER/slam_astar_project/config/slam_params.yaml

Terminal 6 - start RVIZ :

rviz2 -d ~/slam_astar_project/rviz/slam_astar.rviz --ros-args -p use_sim_time:=true

Terminal 7 - use teleop to drive and make the SLAM map :

ros2 run teleop_twist_keyboard teleop_twist_keyboard

Saving the map :
mkdir -p ~/slam_astar_project/maps
ros2 run nav2_map_server map_saver_cli -f ~/slam_astar_project/maps/slam_map

