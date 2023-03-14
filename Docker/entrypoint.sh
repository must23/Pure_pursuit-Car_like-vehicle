#!/bin/bash
 
# Source ROS and Catkin workspaces
source /opt/ros/noetic/setup.bash
if [ -f /TIII_ws/devel/setup.bash ]
then
  source /TIII_ws/devel/setup.bash
fi

echo "Sourced Catkin workspace!"
# Execute the command passed into this entrypoint
exec "$@"
