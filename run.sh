#!/bin/bash

set -e 

cmake --build build -j

./build/simulator &
./build/slam_node
