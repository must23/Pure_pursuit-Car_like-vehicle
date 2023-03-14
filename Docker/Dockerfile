##########################################
# Base Image for GEM_POLARIS Simulation #
#########################################

FROM nvidia/cudagl:11.1.1-base-ubuntu20.04


MAINTAINER Mustofa Basri <mustofabasri23@gmail.com>

# Minimal setup
RUN apt-get update \
 && apt-get install -y locales lsb-release
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

 
# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-noetic-desktop-full
RUN apt-get install -y --no-install-recommends python3-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

#Install additional packages for the dependcies 
RUN  apt install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator

# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# Prepare the colcon workspace
RUN mkdir -p TII_ws/src

WORKDIR /home/$USERNAME/TII_ws

# clone repository
RUN cd /home/$USERNAME/TII_ws/src \
 && git clone https://github.com/osrf/ros_ign.git -b galactic


# Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/noetic/setup.bash \
 && cd TII_ws \
 && catkin_make
RUN echo "source /TII_ws/devel/setup.bash" >> ~/.bashrc


# Copy entry point script, and set the entrypoint
COPY docker/mbzirc_seed/run_solution.bash ./
ENTRYPOINT ["./run_solution.bash"]