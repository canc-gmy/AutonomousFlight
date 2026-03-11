#!/bin/bash

# Get the project root directory (parent of scripts folder)
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting PX4 Offboard Control System..."
echo "Project directory: $PROJECT_DIR"
echo "Scripts directory: $SCRIPT_DIR"

# Launch QGroundControl first (if not already running)
if pgrep -f "QGroundControl-x86_64.AppImage" > /dev/null; then
    echo "QGroundControl is already running."
else
    echo "Launching QGroundControl..."
    ~/Downloads/QGroundControl-x86_64.AppImage &
    sleep 2
fi

# Start Gazebo simulator in first tab
gnome-terminal --window --title="Gazebo Simulator" -- bash -c "
    echo 'Starting Gazebo Simulator...';
    cd $SCRIPT_DIR;
    ./start_gazebo_new.sh;
    exec bash
"

# Wait a bit for Gazebo to start
sleep 3

# Start uORB translator client in second tab
gnome-terminal --tab --title="uORB Translator" -- bash -c "
    echo 'Starting uORB Translator Client...';
    cd $SCRIPT_DIR;
    ./start_uorb_client.sh;
    exec bash
"

# Wait a bit for uORB client to connect
sleep 2

# Build and start the project in third tab
gnome-terminal --tab --title="Offboard Control" -- bash -c "
    echo 'Building and starting Offboard Control...';
    cd $SCRIPT_DIR;
    ./start_project.sh;
    exec bash
"

Start bridge in fourth tab
gnome-terminal --tab --title="Camera Bridge" -- bash -c "
    cd $SCRIPT_DIR;
    ./start_bridge.sh;
    exec bash
"

# Start RViz in fifth tab
# gnome-terminal --tab --title="RViz" -- bash -c "
#     echo 'Launching RViz helper script...';
#     cd $SCRIPT_DIR;
#     ./start_rviz.sh;
#     exec bash
# "


echo "All components started in separate terminals."
echo "Check each terminal window for status."

# Keep the terminal open
exec bash
