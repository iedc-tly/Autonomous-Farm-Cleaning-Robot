#!/usr/bin/env bash
# ==============================================================================
# run_navigation_system.sh
# 
# Complete bringup script for Autonomous Navigation using Nav2:
#   1. Motor Controller + Encoder Odometry + RPLiDAR + Robot State Publisher
#   2. Nav2 Stack (Map Server loading my_map.yaml, AMCL localization, Costmaps,
#      Path Planner, DWB Local Controller, Lifecycle Manager)
#   3. RViz2 Visualization (Map, Costmaps, Particle Cloud, 2D Goal Pose tool)
# ==============================================================================

set -e

# Workspace path
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"
MAP_FILE="${1:-/home/iedctly/my_map.yaml}"

# Print colored header
echo -e "\033[1;34m====================================================================\033[0m"
echo -e "\033[1;32m      AUTONOMOUS ROBOT - NAV2 NAVIGATION & LOCALIZATION SYSTEM      \033[0m"
echo -e "\033[1;34m====================================================================\033[0m"

# 1. Source ROS 2 and Workspace
if [ -f "$ROS_DISTRO_PATH/setup.bash" ]; then
    source "$ROS_DISTRO_PATH/setup.bash"
else
    echo -e "\033[1;31m[ERROR]\033[0m ROS 2 Humble setup not found at $ROS_DISTRO_PATH/setup.bash"
    exit 1
fi

if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo -e "\033[1;33m[WARNING]\033[0m install/setup.bash not found. Building workspace..."
    (cd "$WS_DIR" && colcon build --symlink-install)
    source "$WS_DIR/install/setup.bash"
fi

if [ ! -f "$MAP_FILE" ]; then
    echo -e "\033[1;31m[ERROR]\033[0m Map file not found at: $MAP_FILE"
    exit 1
fi

# Locate RViz configuration
RVIZ_CONFIG="$WS_DIR/src/motor_control/config/mapping.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$WS_DIR/install/motor_control/share/motor_control/config/mapping.rviz"
fi

# Track child PIDs for cleanup
PIDS=()

kill_tree() {
    pkill -9 -f "nav2" 2>/dev/null || true
    pkill -9 -f "amcl" 2>/dev/null || true
    pkill -9 -f "map_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "rplidar_node" 2>/dev/null || true
    pkill -9 -f "motor_node" 2>/dev/null || true
    pkill -9 -f "encoder_odom_node" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "nav2_bringup" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
}

cleanup() {
    echo -e "\n\033[1;33m[SHUTDOWN]\033[0m Stopping all navigation nodes and visualization..."
    
    # Send emergency zero-velocity command
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 || true

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done

    sleep 1
    kill_tree
    echo -e "\033[1;32m[DONE]\033[0m Navigation stack shut down cleanly.\n"
}

trap cleanup SIGINT SIGTERM EXIT

# Kill any leftover zombie nodes before starting
kill_tree


# 2. Launch Nav2 Bringup (Motors, Encoders, RPLiDAR, Map Server, AMCL, Costmaps, Planner/Controller)
echo -e "\033[1;36m[1/2]\033[0m Launching Nav2 Navigation stack with map: $MAP_FILE ..."
ros2 launch motor_control nav2_bringup.launch.py map:="$MAP_FILE" > /tmp/nav2_bringup.log 2>&1 &
NAV2_PID=$!
PIDS+=($NAV2_PID)

echo "      -> Nav2 Bringup started (PID: $NAV2_PID, log: /tmp/nav2_bringup.log)"
echo "      -> Waiting 6 seconds for Nav2 lifecycle nodes to activate..."
sleep 6

# 3. Launch RViz2 Visualization
echo -e "\033[1;36m[2/2]\033[0m Launching RViz2 visualization..."
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz2.log 2>&1 &
else
    rviz2 > /tmp/rviz2.log 2>&1 &
fi
RVIZ_PID=$!
PIDS+=($RVIZ_PID)
echo "      -> RViz2 started (PID: $RVIZ_PID, log: /tmp/rviz2.log)"

echo -e "\033[1;32m--------------------------------------------------------------------\033[0m"
echo -e "\033[1;32m Nav2 System Running! Use '2D Pose Estimate' in RViz2 to localize,  \033[0m"
echo -e "\033[1;32m and '2D Goal Pose' to set navigation targets. Press Ctrl+C to stop.\033[0m"
echo -e "\033[1;32m--------------------------------------------------------------------\033[0m"

wait $NAV2_PID
