#!/bin/bash
source /opt/ros/melodic/setup.bash
source /root/vacuum_ws/devel/setup.bash
roslaunch robonomics_vacuum vacuum_run.launch address:=$1 token:=$2