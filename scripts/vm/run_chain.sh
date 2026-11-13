#!/bin/bash
# Runs several benchmarks one after the other through run_benchmark.sh.
# One line per finished run in ~/CHAIN.progress, ~/CHAIN.done at the end.
#
#   usage: run_chain.sh CHAIN "NAME|benchmark.py args" ["NAME|benchmark.py args" ...]
chain=$1; shift
here=$(dirname "$0")
rm -f ~/"$chain".done ~/"$chain".progress
for item in "$@"; do
  name=${item%%|*}; args=${item#*|}
  "$here"/run_benchmark.sh "$name" $args
  echo "$name $(cat ~/"$name".done)" >> ~/"$chain".progress
done
echo finished > ~/"$chain".done
