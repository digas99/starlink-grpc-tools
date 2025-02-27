#!/bin/bash
# Source the ROS 2 setup script
source /opt/ros/galactic/setup.bash

PUB_RATE=0.1
exec python3 $@ --pub_rate $PUB_RATE