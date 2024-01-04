#!/usr/bin/env python3
"""Installation helpers for the Clearpath Robotics' IndoorNav software

This module contains shared functionality for setting up IndoorNav systemd jobs on
all supported platforms, including Husky, Jackal, Ridgeback, Dingo-O, and Dingo-D.
Other platforms may be added in the future, as needed.

@year 2023
@author Chris Iverach-Brereton <civerachb@clearpathrobotics.com>
"""

import getpass
import os
import robot_upstart
import sys

## Contents for a shell script that waits until we can log into the remote host before starting remote ROS nodes
#
#  Contains 3 variables that must be replaced using .format:
#   - local_user
#   - remote_user
#   - backpack_ip
WAIT_SCRIPT_CONTENTS="""#!/bin/bash

while true;
do
  # SSH credentials are assumed to already be configured, as otherwise we cannot start the remote ROS nodes
  # note that we must run this as the administrator user, not as root!
  echo "[ROS ] Waiting for {backpack_ip} to come up..."
  REMOTE_HOSTNAME=$(sudo -u {local_user} ssh {remote_user}@{backpack_ip} -oHostKeyAlgorithms='ssh-rsa' 'hostname')
  if [ -z "$REMOTE_HOSTNAME" ];
  then
    # wait 5s and try again
    echo -n "."
    sleep 5
  else
    echo "[ROS ] Successfully logged into $REMOTE_HOSTNAME as {remote_user}; can now launch indoornav"
    break
  fi
done
"""

def create_indoornav_service(
    platform,
    local_user="administrator",
    remote_user="administrator",
    backpack_ip="10.252.252.1",
    extra_launch=[]
):
    """Creates the cpr-indoornav.service systemd job for the given platform

    @param platform  A string indicating the supported platform to install. Must be one of
                     - dingo
                     - husky
                     - jackal
                     - ridgeback

    @param local_user  The username of the local user that runs the `ros.service` job
    @param remote_user  The username of the ROS user on the IndoorNav computer
    @param backpack_ip  The IP address of the IndoorNav computer
    @param extra_launch  An optional list of the format [ [pkg1_name, launch_file], [pkg2_name, launch_file], ...]
                         indicating additional launch files to install as part of the systemd job
    """
    try:
        jobname = 'cpr-indoornav'
        j = robot_upstart.Job(name=jobname, workspace_setup=os.environ['ROBOT_SETUP'])
        j.symlink = False
        j.roslaunch_wait = True

        # add launch files from the platform package to the systemd job
        pkg_name = f'cpr_indoornav_{platform}'
        launch_files = [
            f'{platform}_indoornav.launch',   # will be renamed! see below
            'safety_stop.launch',
            'platform_adaptor.launch',
            'power_monitor.launch',
            'indoornav_imu.launch',
            'indoornav_wireless.launch',
            'bridge_relay.launch'
        ]
        for lf in launch_files:
            j.add(package=pkg_name, filename=f'launch/{lf}')

        if extra_launch:
            for launch in extra_launch:
                try:
                    j.add(package=launch[0], filename=launch[1])
                except Exception as err:
                    print(f"Failed to add {launch} to job: {err}")

        j.install()

        # rename the remote roslaunch so it's alphabetically last
        # otherwise the amalgamated launch file may incorrectly start local nodes on the remote host
        os.system(f"sudo mv /etc/ros/noetic/cpr-indoornav.d/{platform}_indoornav.launch /etc/ros/noetic/cpr-indoornav.d/zzzz_{platform}_indoornav.launch")

        # modify the systemd job to use the wait script (installed below) and start after ros.service
        os.system(f"sudo sed -i '/After/c After=ros.service' /lib/systemd/system/{jobname}.service")
        os.system(f"sudo sed -i '/After/a PartOf=ros.service' /lib/systemd/system/{jobname}.service")
        os.system(f"sudo sed -i '/^\[Service\]/a TimeoutStartSec=120' /lib/systemd/system/{jobname}.service")
        os.system(f"sudo sed -i '/^\[Service\]/a ExecStartPre=/usr/sbin/cpr-indoornav-wait' /lib/systemd/system/{jobname}.service")

        # create a wait script to block starting the job until the remote host has booted
        os.system(f'''sudo bash -c "cat > /usr/sbin/cpr-indoornav-wait" << 'EOL'
{WAIT_SCRIPT_CONTENTS}
EOL'''.format(
          backpack_ip=backpack_ip,
          remote_user=remote_user,
          local_user=local_user
        ))
        os.system("sudo chmod +x /usr/sbin/cpr-indoornav-wait")

    except Exception as err:
        print(f"[ERR ] Failed to create cpr-indoornav.service: {err}")
