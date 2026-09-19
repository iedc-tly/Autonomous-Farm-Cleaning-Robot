#!/usr/bin/env bash
# ==============================================================================
# run_mapping_system.sh
# 
# Complete bringup script for:
#   1. RPLiDAR C1 Driver + Motor Controller + Encoder Odometry + TF + SLAM Toolbox
#   2. RViz2 Visualization (Map, Lidar Scan, Odometry, TF)
#   3. Interactive Keyboard Teleoperation (WASD / Arrows)
# ==============================================================================

set -e

# Workspace path
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"

# Print colored header
echo -e "\033[1;34m====================================================================\033[0m"
echo -e "\033[1;32m      AUTONOMOUS ROBOT - SLAM MAPPING & TELEOP BRINGUP SYSTEM       \033[0m"
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

# Locate RViz configuration
RVIZ_CONFIG="$WS_DIR/src/motor_control/config/mapping.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$WS_DIR/install/motor_control/share/motor_control/config/mapping.rviz"
fi

# Track child PIDs for cleanup
PIDS=()

kill_tree() {
    pkill -9 -f "async_slam_toolbox_node" 2>/dev/null || true
    pkill -9 -f "rplidar_node" 2>/dev/null || true
    pkill -9 -f "motor_node" 2>/dev/null || true
    pkill -9 -f "encoder_odom_node" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "slam_bringup" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "teleop" 2>/dev/null || true
}

cleanup() {
    echo -e "\n\033[1;33m[SHUTDOWN]\033[0m Stopping all robot nodes and visualization..."
    
    # Send emergency zero-velocity command to stop any residual motion
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 || true

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -SIGINT "$pid" 2>/dev/null || true
        fi
    done

    sleep 1
    kill_tree
    echo -e "\033[1;32m[DONE]\033[0m All subsystems shut down cleanly.\n"
}

trap cleanup SIGINT SIGTERM EXIT

# Kill any leftover zombie / orphan nodes from previous runs before starting
kill_tree


# 2. Launch SLAM Bringup (Motors, Encoders/Odom, RPLiDAR, Static TF, SLAM Toolbox)
echo -e "\033[1;36m[1/3]\033[0m Launching SLAM bringup stack..."
ros2 launch motor_control slam_bringup.launch.py > /tmp/slam_bringup.log 2>&1 &
SLAM_PID=$!
PIDS+=($SLAM_PID)

echo "      -> SLAM Bringup started (PID: $SLAM_PID, log: /tmp/slam_bringup.log)"
echo "      -> Waiting 4 seconds for nodes to initialize topics..."
sleep 4

# 3. Launch RViz2
echo -e "\033[1;36m[2/3]\033[0m Launching RViz2 visualization..."
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz2.log 2>&1 &
else
    rviz2 > /tmp/rviz2.log 2>&1 &
fi
RVIZ_PID=$!
PIDS+=($RVIZ_PID)
echo "      -> RViz2 started (PID: $RVIZ_PID, log: /tmp/rviz2.log)"
sleep 1

# 4. Launch Interactive Keyboard Teleoperation in Foreground
echo -e "\033[1;36m[3/3]\033[0m Starting keyboard teleoperation in foreground..."
echo -e "\033[1;32m--------------------------------------------------------------------\033[0m"
echo -e "\033[1;32m  Use WASD / Arrow Keys to drive. Press [ Ctrl+C ] to exit safely.  \033[0m"
echo -e "\033[1;32m--------------------------------------------------------------------\033[0m"

ros2 run motor_control teleop
