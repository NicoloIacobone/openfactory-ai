#!/bin/bash
# Runs scripts/benchmark.py in the VM with ROS and the workspace sourced, which a process started
# with `setsid nohup` over SSH does not inherit. Output in ~/NAME.log, exit code in ~/NAME.done.
#
#   usage: run_benchmark.sh NAME [benchmark.py args...]
#   e.g.   ssh doppelvm 'setsid nohup /mnt/openfactory-ai/scripts/vm/run_benchmark.sh p5_eval \
#            --config scripts/config/phase5_tumbled.yaml --set eval > /dev/null 2>&1 < /dev/null &'
#          then wait for ~/p5_eval.done.
name=$1; shift
repo=$(cd "$(dirname "$0")/../.." && pwd)
rm -f ~/"$name".done
source /opt/ros/jazzy/setup.bash
source ~/openfactory_ws/install/setup.bash
cd "$repo"
python3 scripts/benchmark.py "$@" > ~/"$name".log 2>&1 < /dev/null
echo "exit $?" > ~/"$name".done
