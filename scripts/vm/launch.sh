#!/bin/bash
# Runs a `ros2 launch` in the VM with ROS, the workspace and the display of the desktop session
# (the cameras render on it even when headless, D-005). Output in ~/NAME.log.
#
#   usage: launch.sh NAME <ros2 launch arguments...>
#   e.g.   ssh doppelvm 'setsid nohup /mnt/openfactory-ai/scripts/vm/launch.sh ring \
#            openfactory_bringup ring.launch.py n:=2 rviz:=false > /dev/null 2>&1 < /dev/null &'
#          stop it with scripts/vm/stop_cell.sh.
name=$1; shift
source /opt/ros/jazzy/setup.bash
source ~/openfactory_ws/install/setup.bash
export DISPLAY=:0
export XAUTHORITY=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
exec ros2 launch "$@" > ~/"$name".log 2>&1 < /dev/null
