#!/bin/bash

MAX_RETRIES=3
RETRY_COUNT=0
DELAY_BETWEEN_RETRIES=10  # seconds
FAILURE_MSG="Failed to bring up all requested nodes. Aborting bringup."
MAP_FILE="/home/hello-robot/stretch_user/maps/howie_m_final_modified.yaml"
LOG_DIR="/home/hello-robot/nav2_monitor_logs"
mkdir -p "$LOG_DIR"


while [ $RETRY_COUNT -lt $MAX_RETRIES ]; do
    TIMESTAMP=$(date +"%d-%m_%H-%M")
    LOG_FILE="$LOG_DIR/navigation_$TIMESTAMP.txt"

    echo "Launching nav2 (attempt $((RETRY_COUNT+1)))..."
    ros2 launch stretch_nav2 navigation.launch.py map:=$MAP_FILE > "$LOG_FILE" 2>&1 &
    LAUNCH_PID=$!
    echo "Launch PID: $LAUNCH_PID"

    LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')

    echo "Launch PGID: $LAUNCH_PGID"

    # Wait for up to 10 seconds for initial output
    timeout=10
    waited=0
    while [ $waited -lt $timeout ]; do
        if [ -s "$LOG_FILE" ]; then
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done


    # Monitor for failure message
    # This loop keeps running as long as the launch process is alive.
    while kill -0 $LAUNCH_PID 2>/dev/null; do
    #Checks if the failure message has appeared in the log file.
        if grep -q "$FAILURE_MSG" "$LOG_FILE"; then
            echo "Failure detected: $FAILURE_MSG"
            echo "Killing launch group with PGID -$LAUNCH_PGID"
            kill -- -$LAUNCH_PGID 2>/dev/null
            wait $LAUNCH_PID 2>/dev/null
            break
        fi
        sleep 1
    done

    # If the launch process is gone, check if it was due to failure
    if grep -q "$FAILURE_MSG" "$LOG_FILE"; then
        RETRY_COUNT=$((RETRY_COUNT + 1))
        echo "Retrying in $DELAY_BETWEEN_RETRIES seconds..."
        sleep $DELAY_BETWEEN_RETRIES
    else
        echo "nav2 started successfully (or ended cleanly)."
        exit 0
    fi
done

echo "Maximum retries ($MAX_RETRIES) reached. Exiting."
exit 1
