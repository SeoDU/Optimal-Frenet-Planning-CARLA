cmake_minimum_required(VERSION 3.0.2)
project(ee585_vehicle_gen)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  carla_ad_agent
  carla_ego_vehicle
  carla_manual_control
  carla_ros_bridge
  carla_ros_scenario_runner
  carla_spectator_camera
  carla_twist_to_control
  carla_waypoint_publisher
  roscpp
  rospy
  rostopic
  rviz
  rviz_carla_plugin
)

roslaunch_add_file_check(launch)

catkin_package()

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
