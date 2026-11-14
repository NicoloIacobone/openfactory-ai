#!/bin/bash
# Runs the Rubik's cube benchmark in the VM, one cube per launch of the cell: the Gazebo
# server leaks memory at every change of axis of the cube (docs/environment.md), so the cell
# is relaunched for every cube. Output in ~/NAME.log, one line per cube in ~/NAME.progress,
# exit code in ~/NAME.done; the run directory is ~/openfactory_data/trials/NAME.
#
#   usage: run_rubik.sh NAME PART SET FIRST LAST [rubik_benchmark.py args...]
#   e.g.   ssh doppelvm 'setsid nohup /mnt/openfactory-ai/scripts/vm/run_rubik.sh \
#            p7_solve_eval solve eval 0 9 > /dev/null 2>&1 < /dev/null &'
#          then wait for ~/p7_solve_eval.done.
name=$1; part=$2; set=$3; first=$4; last=$5; shift 5
repo=$(cd "$(dirname "$0")/../.." && pwd)
rm -f ~/"$name".done ~/"$name".progress
source /opt/ros/jazzy/setup.bash
source ~/openfactory_ws/install/setup.bash
[ -f ~/openfactory_ws/solver_path ] && export PYTHONPATH="$(cat ~/openfactory_ws/solver_path):$PYTHONPATH"
cd "$repo"
failed=0
for index in $(seq "$first" "$last"); do
  "$repo"/scripts/vm/stop_cell.sh > /dev/null 2>&1
  setsid nohup "$repo"/scripts/vm/launch.sh rubik openfactory_bringup rubik.launch.py \
    rviz:=false > /dev/null 2>&1 < /dev/null &
  sleep 35
  python3 scripts/rubik_benchmark.py --part "$part" --set "$set" --index "$index" \
    --run-id "$name" "$@" >> ~/"$name".log 2>&1 < /dev/null
  code=$?
  [ "$code" -ne 0 ] && failed=$((failed + 1))
  echo "cube $index exit $code" >> ~/"$name".progress
done
"$repo"/scripts/vm/stop_cell.sh > /dev/null 2>&1
python3 scripts/rubik_benchmark.py --summarize --run-id "$name" >> ~/"$name".log 2>&1
echo "exit $failed" > ~/"$name".done
