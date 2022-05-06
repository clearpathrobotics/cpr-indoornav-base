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


Enabling ROS2 to ROS1 Bridge
------------------------------

By default specific topics from the base robot are exposed to ROS2 on domain ID 121.  These include sensor data,
velocity control, battery status, and wireless status.

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
