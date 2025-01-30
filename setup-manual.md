# ROS2 Docker Container Setup Manual

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Platform-Specific Setup](#platform-specific-setup)
3. [Building and Running with Docker Compose](#building-and-running-with-docker-compose)
4. [Manual Build and Run](#manual-build-and-run)
5. [Using the Container](#using-the-container)
6. [Troubleshooting](#troubleshooting)

## Prerequisites

### Common Requirements (All Platforms)
- Docker installed and running
- Git (for cloning the repository)
- Terminal access
- The project source code in the `src` directory

### Checking Docker Installation
```bash
# Verify Docker installation
docker --version

# Verify Docker can run
docker run hello-world
```

## Platform-Specific Setup

### Linux Setup

#### NVIDIA Driver Requirements (for systems with NVIDIA GPUs)
The container requires NVIDIA drivers for GPU support on Linux:

```bash
# Check if NVIDIA drivers are installed
nvidia-smi

# If not installed, install NVIDIA drivers (Ubuntu)
sudo ubuntu-drivers autoinstall
sudo reboot
```

#### NVIDIA Container Toolkit
Required for GPU support in Docker containers:

```bash
# Configure the production repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update and install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### MacOS Setup
No additional setup is required for MacOS as we're using a VNC-based solution. The container will run a VNC server that can be accessed through your web browser.

## Building and Running with Docker Compose

### Using Make Commands
The project includes a Makefile with useful commands for managing the container:

```bash
# Start the container
make start-container

# Connect to the container's shell
make connect-container

# Launch RViz2 in the container
make launch-rviz2

# Stop the container
make stop-container

# Rebuild and start the container
make rebuild-container
```

### Manual Docker Compose Commands
If you prefer not to use Make:

```bash
# Start the container
docker compose up -d

# Stop the container
docker compose down

# Rebuild and start
docker compose up --build -d
```

## Manual Build and Run

### Building the Docker Image

#### Linux Build
```bash
# Change directory to scripts
cd scripts/
# Run the build script
./build_linux.sh
```

The Linux build uses the OSRF ROS2 desktop-full image and includes NVIDIA GPU support.

#### MacOS Build
```bash
# Change directory to scripts
cd scripts/
# Run the build script
./build_mac.sh
```

The MacOS build uses the ROS2 desktop VNC image, which includes a built-in VNC server.

## Using the Container

### Accessing the GUI

#### Linux Users
GUI applications will work directly through X11 forwarding:
```bash
# Test RViz
ros2 run rviz2 rviz2

# Test Gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

#### MacOS Users
1. Start the container using docker compose or the provided scripts
2. Open your web browser and navigate to:
   ```
   http://localhost:6080
   ```
3. You'll see the VNC interface in your browser, where you can run GUI applications

### Common Operations

```bash
# Connect to the container's shell
docker exec -it ros2_desktop_vnc /bin/bash

# Build the workspace
cd /root/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --mixin release

# Source the workspace
source /root/colcon_ws/install/setup.bash
```

## Troubleshooting

### Common Issues on Linux

1. NVIDIA GPU Issues:
```bash
# Check NVIDIA driver status
nvidia-smi

# Verify Docker NVIDIA support
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

2. Display Issues:
```bash
# Reset X server permissions
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY
```

### Common Issues on MacOS

1. VNC Connection Issues:
- Ensure port 6080 is not being used by another application
- Check if the container is running:
  ```bash
  docker ps | grep ros2_desktop_vnc
  ```
- Verify port mapping:
  ```bash
  docker compose ps
  ```

2. Performance Issues:
- The VNC connection uses port 6080 by default
- Increase the `shm_size` in docker-compose.yml if you experience performance issues
- Consider adjusting the VNC quality settings in the browser interface

### General Troubleshooting
If you encounter issues:
1. Check container logs:
   ```bash
   docker compose logs
   ```
2. Verify the container is running:
   ```bash
   docker ps
   ```
3. Ensure all volumes are properly mounted:
   ```bash
   docker compose config
   ```
4. Try rebuilding the container:
   ```bash
   make rebuild-container
   ```

## Notes
- Keep the source directory updated
- Rebuild the container if Dockerfile changes
- For MacOS users: Access the GUI through the VNC web interface
- For Linux users: Ensure NVIDIA drivers are up to date if using GPU
