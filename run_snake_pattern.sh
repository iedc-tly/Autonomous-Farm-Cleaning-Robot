#!/usr/bin/env bash
# ==============================================================================
# run_snake_pattern.sh
# 
# Tool for:
#   1. Recording points clicked using RViz '2D Goal Pose' tool (/goal_pose)
#   2. Generating snake (lawnmower / boustrophedon) path waypoints
#   3. Executing waypoint navigation through Nav2
# ==============================================================================

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_PATH="/opt/ros/humble"

# Source ROS 2 and workspace
source "$ROS_DISTRO_PATH/setup.bash"
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
fi

echo -e "\033[1;34m====================================================================\033[0m"
echo -e "\033[1;32m       SNAKE PATTERN COVERAGE & WAYPOINT NAVIGATION MANAGER        \033[0m"
echo -e "\033[1;34m====================================================================\033[0m"
echo -e "Choose an option:"
echo -e "  \033[1;33m[1]\033[0m Record goal poses from RViz (/goal_pose)"
echo -e "  \033[1;33m[2]\033[0m Record goal poses in RViz to define Area & generate Snake pattern"
echo -e "  \033[1;33m[3]\033[0m Auto-generate Snake (Lawnmower) grid pattern from manual Area Bounds"
echo -e "  \033[1;33m[4]\033[0m Execute Snake Waypoint Navigation on Robot"
echo -e "  \033[1;33m[5]\033[0m Exit"
echo -e "\033[1;34m====================================================================\033[0m"

read -p "Select [1-5]: " CHOICE

case $CHOICE in
    1)
        read -p "Output YAML file path [default: waypoints.yaml]: " OUTFILE
        OUTFILE=${OUTFILE:-waypoints.yaml}
        echo -e "\n\033[1;36mStarting Pose Recorder... Click '2D Goal Pose' in RViz to add points.\033[0m"
        python3 "$WS_DIR/src/motor_control/motor_control/snake_pattern_navigator.py" record -o "$OUTFILE"
        ;;
    2)
        read -p "Number of goal poses to record [default: 2 (opposite corners)]: " NUM_POSES
        NUM_POSES=${NUM_POSES:-2}
        read -p "Step size / grid resolution in meters [default: 0.5m, press Enter]: " STEP_INPUT
        STEP_INPUT=${STEP_INPUT:-0.5}
        STEPX=$STEP_INPUT
        STEPY=$STEP_INPUT
        read -p "Output YAML file path [default: snake_waypoints.yaml]: " OUTFILE
        OUTFILE=${OUTFILE:-snake_waypoints.yaml}
        read -p "Execute navigation directly after generating? [y/N]: " EXEC_CHOICE

        EXEC_FLAG=""
        if [[ "$EXEC_CHOICE" =~ ^[Yy]$ ]]; then
            EXEC_FLAG="--execute"
        fi

        echo -e "\n\033[1;36mStarting Area Pose Recorder... Click $NUM_POSES '2D Goal Pose' point(s) in RViz to define the area.\033[0m"
        python3 "$WS_DIR/src/motor_control/motor_control/snake_pattern_navigator.py" record-area \
            --num-poses "$NUM_POSES" --step-x "$STEPX" --step-y "$STEPY" -o "$OUTFILE" $EXEC_FLAG
        ;;
    3)
        read -p "Enter Min X (meters): " XMIN
        read -p "Enter Max X (meters): " XMAX
        read -p "Enter Min Y (meters): " YMIN
        read -p "Enter Max Y (meters): " YMAX
        read -p "Step size / grid resolution in meters [default: 0.5m, press Enter]: " STEP_INPUT
        STEP_INPUT=${STEP_INPUT:-0.5}
        STEPX=$STEP_INPUT
        STEPY=$STEP_INPUT
        read -p "Output YAML file path [default: snake_waypoints.yaml]: " OUTFILE
        OUTFILE=${OUTFILE:-snake_waypoints.yaml}

        python3 "$WS_DIR/src/motor_control/motor_control/snake_pattern_navigator.py" generate \
            --xmin "$XMIN" --xmax "$XMAX" --ymin "$YMIN" --ymax "$YMAX" \
            --step-x "$STEPX" --step-y "$STEPY" -o "$OUTFILE"
        ;;
    4)
        read -p "Input Waypoints YAML file path [default: snake_waypoints.yaml]: " INFILE
        INFILE=${INFILE:-snake_waypoints.yaml}
        if [ ! -f "$INFILE" ]; then
            echo -e "\033[1;31m[ERROR]\033[0m File $INFILE does not exist!"
            exit 1
        fi
        echo -e "\n\033[1;36mLaunching Waypoint Executor... Make sure run_navigation_system.sh is running!\033[0m"
        python3 "$WS_DIR/src/motor_control/motor_control/snake_pattern_navigator.py" execute -i "$INFILE"
        ;;
    5)
        echo "Exiting."
        exit 0
        ;;
    *)
        echo "Invalid selection."
        exit 1
        ;;
esac

