FROM tiryoh/ros2-desktop-vnc:jazzy

# Update and upgrade the system
RUN apt-get update && apt-get dist-upgrade -y

# Install additional packages
RUN apt-get update && apt-get install -y \
build-essential \
git \
wget \
vim \
python3-pip \
python3-rosdep \
python3-colcon-common-extensions \
ros-jazzy-ros2-control \
ros-jazzy-ros2-controllers \
ros-jazzy-controller-manager \
ros-jazzy-joint-state-broadcaster \
ros-jazzy-position-controllers \
ros-jazzy-moveit-py \
ros-jazzy-moveit \
ros-jazzy-moveit-hybrid-planning \
ros-jazzy-xacro \
ros-jazzy-ompl \
ros-jazzy-joint-state-publisher-gui \
ros-jazzy-vision-opencv \
ros-jazzy-image-transport \
ros-jazzy-cv-bridge \
ros-jazzy-image-pipeline \
&& rm -rf /var/lib/apt/lists/*

# Source ROS 2 automatically in container
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
