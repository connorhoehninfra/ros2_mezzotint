# Use the official ROS 2 Jazzy desktop full image
FROM osrf/ros:jazzy-desktop-full

# Conditional Flag to specify if the host is a Mac or not
ARG ON_MAC=false

# Install common necessary packages
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    wget \
    vim \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions

# Install Mac-specific packages if ON_MAC is true
RUN if [ "$ON_MAC" = "true" ]; then \
        apt-get install -y \
        x11-apps \
        mesa-utils \
        xauth; \
    fi

# Install Moveit2 and other packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-moveit* \
    ros-jazzy-ompl* \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \ 
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-position-controllers 

# Create a workspace
RUN mkdir -p /root/colcon_ws/src 

# Copy src folder
COPY src /root/colcon_ws/src

# Build workspace
RUN cd /root/colcon_ws && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --mixin release"

# Set display environment variable for Mac
RUN if [ "$ON_MAC" = "true" ]; then \
        echo "export DISPLAY=host.docker.internal:0" >> /root/.bashrc; \
    fi

# Set the working directory
WORKDIR /root/colcon_ws

# Copy entrypoint script
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]