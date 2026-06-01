#!/bin/bash

set -e 

cmake --build build -j
trap 'kill 0' EXIT SIGINT

./build/simulator &
SIM_PID=$!

sleep 0.1

./build/slam_node &
SLAM_PID=$!

wait $SLAM_PID
# wait $SIM_PID $SLAM_PID


