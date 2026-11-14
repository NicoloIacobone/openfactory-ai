#!/bin/bash
# Stops whatever cell or ring is running in the VM: the launch process first, then the nodes it
# may leave behind (a launch that is killed or aborts orphans `gz sim` and the spawners).
# Runs as a file on purpose: the same `pkill -f` typed inside `ssh doppelvm '...'` would match
# the remote shell's own command line and kill the SSH session (docs/environment.md).
#
#   usage: ssh doppelvm /mnt/openfactory-ai/scripts/vm/stop_cell.sh
patterns=(
  'ros2 launch openfactory_'
  'gz sim'
  'ros_gz_bridge/parameter_bridge'
  'robot_state_publisher/robot_state_publisher'
  'moveit_ros_move_group/move_group'
  'openfactory_simulation/depth_sensor_model'
  'openfactory_perception/scene_perception'
  'openfactory_perception/cube_perception'
  'controller_manager/spawner'
  'rviz2/rviz2'
)
pkill -TERM -f "${patterns[0]}"
sleep 5
for pattern in "${patterns[@]}"; do
  pkill -KILL -f "$pattern"
done
sleep 1
left=0
for pattern in "${patterns[@]}"; do
  if pgrep -f "$pattern" > /dev/null; then
    echo "still running: $pattern"
    left=1
  fi
done
[ "$left" -eq 0 ] && echo "stopped"
exit "$left"
