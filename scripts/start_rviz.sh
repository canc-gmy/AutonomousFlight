#!/bin/bash

# start_rviz.sh - launches rviz2 with workspace sourcing if available

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Starting RViz..."

# Source the workspace install (if present) so RViz can find ROS topics/types
if [ -f "$PROJECT_DIR/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "$PROJECT_DIR/install/setup.bash" 2>/dev/null || true
fi

if command -v rviz2 >/dev/null 2>&1; then
    rviz2
else
    echo "rviz2 not found in PATH; please install ROS2 RViz or source your ROS2 environment."
    exit 1
fi
