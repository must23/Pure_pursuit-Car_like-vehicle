# tech_assigmnt


This repository provides a simulated vehicle model of the Polaris GEM e2 Electric Cart in the Gazebo simulation environment as well as ROS-based sensors and controllers for autonomous driving. 
Inside gem_drivers_sim, there's a path tracking controller in C++ which given a specific path, can follow this path with a max 20km/h with an accuracy of path
tracking of at least 1m. The path tracking is based on the pure pursuit algorithm where it takes the given coordinate and yaw angle of a path, then tries to track it using the inputs to the controller:
1. Path ros message, with GPS coordinates for the lat and long coordinates that have been transformed to the x and y gazebo coordinate. http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Path.html
2. Odometry ros message for the yaw angle. http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html


## Setup

### Docker Setup
First, install Docker and Docker Compose using [the official install guide](https://docs.docker.com/engine/install/ubuntu/).

To run Docker containers with graphics and GPU support, you will also need the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker).

To use GUI based tools (e.g., RViz, Gazebo) inside Docker, there is additional setup required. The simplest way is to run the command below each time you log into your machine, but there is a more detailed walkthrough of options in the [ROS Wiki](http://wiki.ros.org/docker/Tutorials/GUI).

```
$ xhost + local:docker
```

First, clone this repository and go into the top-level folder:

```
$ git clone https://github.com/must23/tech_assigmnt.git
$ cd tech_assigmnt/Docker
```

Build the Docker images. 
```
$ docker build .
```

Run the bash file from the Docker folder.
```
# container 1
$ ./run_docker.sh nvidia_ros "roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true""
"
$ ./run_docker.sh nvidia_ros "roslaunch gem_gazebo gem_sensor_info.launch

$ ./run_docker.sh nvidia_ros "rosrun gem_pure_pursuit_sim pure_pursuit_sim"
```

### Local Setup

If you do not want to use Docker, you can directly clone this package to a catkin_make workspace with the necessary dependencies. As long as you can run the examples in the [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview), you should be in good shape.

First, make a workspace and clone this repo there:

```
mkdir TII_ws/src
cd TII_ws/src
git clone https://github.com/must23/tech_assigmnt.git
source /opt/ros/noetic/setup.bash
```

Clone the external dependencies:

```
$ sudo apt install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator
```

Set up any additional dependencies using rosdep:

```
$ sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Then, build the workspace.

```
$ cd ~/TII_ws
$ catkin_make
```

---

## Basic Usage

### Simple Track Environment

```bash
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"

```

<a href="url"><img src="https://github.com/must23/tech_assigmnt/blob/main/media/pp_controller.gif" width="600"></a>

taken from Polaris GEM e2 Simulator GitLab

### Pure Pursuit Path Tracker

```bash
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"

$ source devel/setup.bash
$ roslaunch gem_gazebo gem_sensor_info.launch

$ source devel/setup.bash
$ rosrun gem_pure_pursuit_sim pure_pursuit_sim.py
OR
$ rosrun gem_pure_pursuit_sim pure_pursuit_sim
For C++ implementation

```

## Deliverables
1. Codes and dockerfile are included in this repository under directory:  
```
\polaris_gem_drivers_sim\gem_pure_pursuit_sim\src 
and 
\polaris_gem_drivers_sim\gem_pure_pursuit_sim\script
```
2. A demonstration video can be accessed via this URL: 
```
https://youtu.be/2quJo050ScU
```
3. An image of the plot showing the cross-track error while path tracking is shown in the figure below.

<a href="url"><img src="https://github.com/must23/tech_assigmnt/blob/main/media/cross-track-error_.png" width="600"></a>



## Known Issues

+ Currently the latest Velodyn LiDAR sensor for Gazebo package (1.0.12) on Ubuntu 20.04,
 `velodyne_simulator`, publishes ROS messages with incorrect `frame_id` field.
  The field is appended with a prefix that is the namespace of the ROS node.
  TF and TF2 has deprecated the `tf_prefix` feature and does not recommand adding the
  prefix, and therefore RViz was not able to visualize the Velodyn LiDAR scan.
  See http://wiki.ros.org/tf2/Migration.
+ As of the current development on the Polaris original project, The sensor providing ground truth Odometry of base_link is based on the gazebo plugin libgazebo_ros_p3d.so. This plugin does not provide tree in the tf despite that it produces the odom data on /gem/base_link/odom topic. This contributes to the rviz issue on the nonexisting odom fixed frame which led to the robot does not move in the rviz (Gazebo moves) even though the wheel is spinning. To compensate, a script to convert and publish odom data into the tf is required.
 
