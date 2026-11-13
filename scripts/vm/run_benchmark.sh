#!/bin/bash
# Runs a benchmark script in the VM with ROS and the workspace sourced, which a process started
# with `setsid nohup` over SSH does not inherit. Output in ~/NAME.log, exit code in ~/NAME.done.
#
#   usage: run_benchmark.sh NAME [--script FILE] [benchmark args...]
#   FILE is a script of scripts/ (default benchmark.py; relay_benchmark.py for the ring).
#   e.g.   ssh doppelvm 'setsid nohup /mnt/openfactory-ai/scripts/vm/run_benchmark.sh p5_eval \
#            --config scripts/config/phase5_tumbled.yaml --set eval > /dev/null 2>&1 < /dev/null &'
#          then wait for ~/p5_eval.done.
name=$1; shift
script=benchmark.py
if [ "$1" = "--script" ]; then
  script=$2; shift 2
fi
repo=$(cd "$(dirname "$0")/../.." && pwd)
rm -f ~/"$name".done
source /opt/ros/jazzy/setup.bash
source ~/openfactory_ws/install/setup.bash
cd "$repo"
python3 scripts/"$script" "$@" > ~/"$name".log 2>&1 < /dev/null
echo "exit $?" > ~/"$name".done
