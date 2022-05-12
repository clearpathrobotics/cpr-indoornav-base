cmake_minimum_required(VERSION 3.0.2)
project(cpr_indoornav_base)

find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)

install(PROGRAMS
  # nodes for connecting to the ROS2 API
  scripts/start_autonomy_bridge
  scripts/start_fleet_bridge

  # scripts to run on the backpack remotely
  scripts/start_api
  scripts/start_navigation

  # scripts for configuring the systemd job, installing dependencies, configuring external tools
  scripts/install_depends
  scripts/setup
  scripts/setup_netplan
  scripts/reconfigure_apache
  scripts/reconfigure_api
  scripts/reconfigure_chrony
  scripts/reconfigure_iptables
  scripts/reconfigure_ros_setup
  scripts/reconfigure_ssh

  # other utilities
  scripts/tf_republisher

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
