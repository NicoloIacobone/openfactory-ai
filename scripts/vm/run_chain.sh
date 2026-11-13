#!/bin/bash
# Runs several benchmarks one after the other through run_benchmark.sh.
# One line per finished run in ~/CHAIN.progress, ~/CHAIN.done at the end.
#
#   usage: run_chain.sh CHAIN "NAME|run_benchmark.sh args" ["NAME|run_benchmark.sh args" ...]
#   e.g.   run_chain.sh p6 "p6_eval_n2|--script relay_benchmark.py --arms 2 --set eval"
chain=$1; shift
here=$(dirname "$0")
rm -f ~/"$chain".done ~/"$chain".progress
for item in "$@"; do
  name=${item%%|*}; args=${item#*|}
  "$here"/run_benchmark.sh "$name" $args
  echo "$name $(cat ~/"$name".done)" >> ~/"$chain".progress
done
echo finished > ~/"$chain".done
