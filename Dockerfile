FROM osrf/ros:jazzy-desktop

# Install Gazebo Harmonic
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-teleop-twist-keyboard \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Create a user 'ros' with the same UID/GID as the host user
# The base image (Ubuntu 24.04) creates a user 'ubuntu' with UID 1000.
# We remove it to avoid conflict if the host user is also 1000.
ARG USERNAME=ros
ARG USER_UID=1001
ARG USER_GID=$USER_UID

ARG RENDER_GID=110

RUN if id -u ubuntu > /dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && groupadd --gid $RENDER_GID render \
    && usermod -aG video,render $USERNAME

# Set up the environment
USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

# Source ROS 2 setup in .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc

CMD ["/bin/bash"]
