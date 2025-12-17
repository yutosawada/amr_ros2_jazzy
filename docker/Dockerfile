FROM osrf/ros:jazzy-desktop

ARG USER_UID=1000
ARG USER_GID=1000
# Default workspace path; kept world-writable so arbitrary UID/GID can work
ARG WORKSPACE=/workspace

# Install Gazebo Harmonic
# Copy dependency files
COPY config/dependencies.yaml .
COPY config/install_dependencies.sh .

# Run dependency installation script
RUN chmod +x install_dependencies.sh && ./install_dependencies.sh

RUN mkdir -p ${WORKSPACE}/ros2_ws && chmod -R 777 ${WORKSPACE}
ENV HOME=${WORKSPACE}
WORKDIR ${WORKSPACE}/ros2_ws

# Source ROS 2 setup in a global shell rc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash"]
