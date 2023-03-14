cmake_minimum_required(VERSION 2.8.3)
project(gem_pure_pursuit_sim)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  ackermann_msgs
  visualization_msgs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(gem_pure_pursuit_sim src/main.cpp src/PurePursuit.cpp src/PurePursuit.h)
add_dependencies(gem_pure_pursuit_sim ${gem_pure_pursuit_sim_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gem_pure_pursuit_sim
   ${catkin_LIBRARIES}
 )
