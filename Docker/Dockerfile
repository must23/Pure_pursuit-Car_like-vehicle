##########################################
# Base Image for GEM_POLARIS Simulation #
#########################################

FROM nvidia/cudagl:11.1.1-base-ubuntu20.04

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

RUN apt-get install -y git
RUN apt-get install -y apt-utils
 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN apt-get install -y libeigen3-dev

RUN apt-get install -y python3-pip git && \
    pip install catkin_tools

RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

#Install additional packages for the dependcies 
RUN  apt-get install -y ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator



# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME
RUN echo source /opt/ros/noetic/setup.bash
# Commands below run as the developer user

RUN mkdir -p TIII_ws/src

# clone repository
RUN cd /home/$USERNAME/TIII_ws/src \
 && git clone https://github.com/must23/tech_assigmnt.git



WORKDIR /home/$USERNAME/TIII_ws


# Build the Catkin workspace and ensure it's sourced

RUN cd /home/$USERNAME/TIII_ws/ 

RUN . /opt/ros/noetic/setup.sh && \
    cd /home/$USERNAME/TIII_ws && \
    catkin build
