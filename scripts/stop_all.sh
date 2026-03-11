#!/bin/bash

echo "Stopping all components..."

# Kill Gazebo processes
killall -g ruby gazebo gz
pkill -f px4

# Kill uORB client
pkill -f micrortps_client

# Kill offboard control
pkill -f offboard_control

# Close all gnome-terminal windows
# pkill -f gnome-terminal

echo "All components stopped."

exec bash

# NOTE: Often, pkill does not correctly close gazebo, that keeps living as a zombie process.
# In that case, you might need to manually kill it using:
# ps aux | grep gz # find the PID of gz 
# kill -9 274535  # replace 274535 with the actual PID 