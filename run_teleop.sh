#!/bin/bash
docker exec -it amr_sim bash -c "source /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
