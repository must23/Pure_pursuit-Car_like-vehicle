#!/bin/bash
 
# Source ROS and Catkin workspaces
source /opt/ros/noetic/setup.bash
if [ -f /TIII_ws/devel/setup.bash ]
then
  source /TIII_ws/devel/setup.bash
fi

echo "Sourced Catkin workspace!"
# Execute the command passed into this entrypoint
roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
roslaunch gem_gazebo gem_sensor_info.launch
rosrun gem_pure_pursuit_sim pure_pursuit_sim
