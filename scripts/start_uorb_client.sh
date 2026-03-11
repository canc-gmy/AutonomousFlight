#!/bin/bash

echo "Starting uORB Translator Client..."

# Wait for PX4 to be ready
echo "Waiting for PX4 to start..."
sleep 5

# Start the MicroXRCE Agent
# Adjust the command based on your specific setup
if command -v MicroXRCEAgent &> /dev/null; then
    MicroXRCEAgent udp4 -p 8888
elif command -v micro-ros-agent &> /dev/null; then
    echo "Found micro-ros-agent, starting..."
    micro-ros-agent udp4 --port 8888
elif [ -f "/opt/px4/bin/MicroXRCEAgent" ]; then
    /opt/px4/bin/MicroXRCEAgent udp4 -p 8888
else
    echo "Error: MicroXRCEAgent not found!"
    echo "Please update this script with the correct path to MicroXRCEAgent."
    exit 1
fi

echo "uORB translator client started."
