#!/bin/bash

echo "=== Stopping containers... ===" && \
docker compose down && \
echo "=== Building containers (no cache)... ===" && \
docker compose build --no-cache && \
echo "=== Starting containers... ===" && \
docker compose up -d && \
echo "=== Waiting for container to start... ===" && \
sleep 3 && \
echo "=== Building ROS2 packages in container... ===" && \
docker exec amr_sim bash -c "source /opt/ros/jazzy/setup.bash && colcon build" && \
echo "=== Done! ===" || \
echo "=== Error occurred! ==="

