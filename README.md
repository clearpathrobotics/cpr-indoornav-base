Clearpath IndoorNav Base
==========================

For the full IndoorNav user manual, please refer to https://clearpathrobotics.com/assets/manuals/indoornav/index.html

This package contains the core launch files for using Clearpath's IndoorNav software.  Five robots are currently
supported, with their own separate repositories:

- Dingo-D: https://github.com/clearpathrobotics/cpr-indoornav-dingo
- Dingo-O: https://github.com/clearpathrobotics/cpr-indoornav-dingo
- Husky: https://github.com/clearpathrobotics/cpr-indoornav-husky
- Jackal: https://github.com/clearpathrobotics/cpr-indoornav-jackal
- Ridgeback: https://github.com/clearpathrobotics/cpr-indoornav-ridgeback

This package must be present on both the robot's main PC and on the backpack PC.


Required Configuration
------------------------

You must remove `eno1` from the `br0` ethernet bridge configure the robot's `eno1` ethernet port to have
a static IP address:
```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eno1:
      ignore-carrier: true
      dhcp4: no
      dhcp6: no
      addresses:
        - 10.252.252.100/24
    bridge_eth:
      dhcp4: no
      dhcp6: no
      match:
        name: eth*
    bridge_enp:
      dhcp4: no
      dhcp6: no
      match:
        name: enp*
    bridge_enx:
      dhcp4: no
      dhcp6: no
      match:
        name: enx*
  bridges:
    br0:
      dhcp4: yes
      dhcp6: no
      interfaces: [bridge_eth, bridge_enp, bridge_enx]
      addresses:
        - 192.168.131.1/24
```

The backpack PC and robot PC must have their `eno1` ports directly connected with an ethernet cable.

For convenience, running
```bash
rosrun cpr_indoornav_base setup_netplan
```
will generate the file above and apply it.

Once the networking has been configured, run
```bash
rosrun cpr_indoornav_base setup
```
to apply the rest of the configuration to the robot's PC.

This script will perform the following actions to configure the communication between the robot's internal PC and
the IndoorNav backpack:

- install additional dependencies if they are not present already
- generate a new, passwordless SSH key in RSA format to `$HOME/.ssh/id_rsa{.pub}`
- add an entry to /etc/hosts for the backpack PC
- configure `apache2` to act as a proxy server for IndoorNav's web GUI
- add additional envars to `/etc/ros/setup.bash`
- configure ``chrony`` to allow NTP connections on ``192.168.131.0/24``, ``10.252.252.0/24`` and ``10.255.255.0/24``

If you need to reconfigure any individual component you can run each section of the setup script independently with
the following scripts:
- `install_depends`
- `reconfigure_apache`
- `reconfigure_chrony`
- `reconfigure_ros_setup`
- `reconfigure_ssh`


Starting and Stopping
-----------------------

To start manually the IndoorNav backpack, wait for both the robot's main PC and backpack to boot.  Then log into the
robot's main PC and run the appropriate command from the following list (assuming you have the corresponding package
already installed)

```bash
roslaunch cpr_indoornav_dingo dingo_indoornav.launch
roslaunch cpr_indoornav_husky husky_indoornav.launch
roslaunch cpr_indoornav_jackal jackal_indoornav.launch
roslaunch cpr_indoornav_ridgeback ridgeback_indoornav.launch
```


Systemd Job Creation
----------------------

To create the `cpr-indoornav` systemd job, run

```bash
rosrun cpr_indoornav_ROBOT install
```

where `ROBOT` is one of the supported platforms:
- `dingo`
- `husky`
- `jackal`
- `ridgeback`

This will create a new systemd job which will start the indoor navigation software automatically when the robot and
backpack PC have finished booting.

To customize the systemd job, refer to the launch files found in `/etc/ros/$ROS_DISTRO/cpr-indoornav.d`.

To manually start/stop/restart the job, use the following commands:

```bash
sudo systemctl start cpr-indoornav
sudo systemctl stop cpr-indoornav
sudo systemctl restart cpr-indoornav
```

Note that if you restart the `ros` systemd job you should also restart the `cpr-indoornav` systemd job, as these
jobs use the same underlying `roscore` process.


Supported Platforms
---------------------

IndoorNav is currently supported on the following Clearpath platforms:

- Dingo-D (Dingo-O support coming soon!)
- Husky
- Jackal
- Ridgeback

Boxer 2.4 includes IndoorNav as part of its core software; this package is not needed for Boxer.

IndoorNav is not supported on Clearpath's outdoor-only platforms, including Warthog, Moose, and Heron.

Legacy platforms, such as Boxer 1.0, Husky A100, Grizzly, and Kingfisher are likewise not supported.


ROS1 to ROS2 Bridge and API Domains
------------------------------------

The IndoorNav API is separated into 3 components, each operating on a different ROS2 domain:

| Domain ID | API      | ROS1 Bridge Node   | Topic Namespace     |
|-----------|----------|--------------------|---------------------|
| 100       | Fleet    | `fleet_bridge`     | `/cpr_fleet_api`    |
| 95        | Autonomy | `autonomy_bridge`  | `/cpr_autonomy_api` |
| 90        | Platform | `platform_bridge`  | `/cpr_platform_api` |

The Platform API is not used for IndoorNav, as it is specific to the OTTO 100 and OTTO 1500.  By default, all
suppored Clearpath platforms (Jackal, Husky, Ridgeback, and Dingo) will bridge specific ROS1 topics, including
sensor data, controller inputs, and ``cmd_vel`` into ROS2 on domain ``91``, using the robot's hostname as a
namespace.


Enabling ROS2 to ROS1 Bridge
------------------------------

To enable exposing the ROS2 Fleet and Autonomy API topics to the ROS1 master you must compile ``ros1_bridge`` from
source.  We recommend using ROS2 Foxy or Galactic.

Create a Colcon workspace and clone the ``ros1_bridge`` and ``clearpath_api`` repositories into it:

```bash
mkdir -p $HOME/bridge_ws/src
cd $HOME/bridge_ws/src

# Clone the ROS1 Bridge source:
git clone https://github.com/ros2/ros1_bridge.git
# Alternatively, this fork contains experimental support for bridging Actions
#git clone https://github.com/ipa-hsd/ros1_bridge/tree/action_bridge.git -b action_server

# TODO - this won't work yet as the repo has not been made public yet
git clone https://github.com/clearpathrobotics/clearpath_api.git -b foxy-devel
```

To build the packages, run the following commands:
```bash
cd $HOME/bridge_ws
export CMAKE_PREFIX_PATH=
export ROS1_INSTALL_PATH=/opt/ros/noetic
export ROS2_INSTALL_PATH=/opt/ros/foxy     # or /opt/ros/galactic

source ${ROS2_INSTALL_PATH}/setup.bash
colcon build --symlink-install --packages-skip ros1_bridge

source ${ROS1_INSTALL_PATH}/setup.bash
source ${ROS2_INSTALL_PATH}/setup.bash

# If you have a catkin_ws with ROS1 packages, source it here
#source /home/administrator/catkin_ws/devel/setup.bash
source $HOME/bridge_ws/install/local_setup.bash

colcon build --symlink-install --cmake-force-configure --allow-overriding ros1_bridge
```

Once the packages are built, enable the ROS2 to ROS1 bridge by adding the following to ``/etc/ros/setup.bash``

```bash
export INDOORNAV_ENABLE_ROS2_TO_ROS1_BRIDGE=1
```

Then restart ROS and the IndoorNav systemd jobs:

```bash
sudo systemctl stop cpr-indoornav
sudo systemctl restart ros
sudo systemctl start cpr-indoornav
```


Building the ROS2 API Examples
-------------------------------

The ROS2 API examples in the published SDK require manual patching to build correctly.

1. Edit `/opt/clearpath/ros2-api-1.3.3/include/clearpath_api/autonomy_api.hpp` to add
```
#include <std_msgs/Float32.hpp>
```
to the includes at the top of the file

2. Replace `opt/clearpath/ros2-api-1.3.3/share/clearpath_api/examples/CMakeLists.txt` with the following:
```
cmake_minimum_required(VERSION 3.5)
project(clearpath_api_examples)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(clearpath_api REQUIRED)
find_package(perception_navigation_msgs REQUIRED)
find_package(command_control_msgs REQUIRED)

ament_package()

add_executable(monitor_odom_intent "src/monitor_odom_intent.cpp")

ament_target_dependencies(monitor_odom_intent
  "rclcpp"
  "clearpath_api"
  "perception_navigation_msgs"
  )

add_executable(send_move_goal "src/send_move_goal.cpp")

ament_target_dependencies(send_move_goal
  "rclcpp"
  "rclcpp_action"
  "clearpath_api"
  "command_control_msgs"
  )

add_executable(get_map_info "src/get_map_info.cpp")

ament_target_dependencies(get_map_info
  "rclcpp"
  "clearpath_api"
  "command_control_msgs"
  "perception_navigation_msgs"
  )

install(TARGETS monitor_odom_intent send_move_goal get_map_info DESTINATION lib/${PROJECT_NAME})
```

