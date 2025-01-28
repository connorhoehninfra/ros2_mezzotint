# ROS2 Docker Container Setup Manual

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Platform-Specific Setup](#platform-specific-setup)
3. [Building the Docker Image](#building-the-docker-image)
4. [Running the Container](#running-the-container)
5. [Troubleshooting](#troubleshooting)

## Prerequisites

### Common Requirements (All Platforms)
- Docker Desktop installed and running
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

#### 1. NVIDIA Driver Requirements (for systems with NVIDIA GPUs)
The container requires NVIDIA drivers for GPU support on Linux:

```bash
# Check if NVIDIA drivers are installed
nvidia-smi

# If not installed, install NVIDIA drivers (Ubuntu)
sudo ubuntu-drivers autoinstall
sudo reboot
```

#### 2. NVIDIA Container Toolkit
Required for GPU support in Docker containers:

```bash
# Configure the production repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update the packages list from the repository 
sudo apt-get update
# Install NVIDIA Container Toolkit
sudo apt-get install -y nvidia-container-toolkit
# Configure the container runtime by using the nvidia-ctk command
sudo nvidia-ctk runtime configure --runtime=docker
# Restart docker daemon
sudo systemctl restart docker
```

### MacOS Setup

#### 1. XQuartz Installation
XQuartz is required for running GUI applications from Docker containers on MacOS:

```bash
# Install using Homebrew
brew install --cask xquartz
```

Why XQuartz is needed:
- MacOS doesn't include a native X11 server
- XQuartz provides the X11 server needed for GUI applications
- Enables forwarding of graphical applications from the Docker container to your Mac's display

#### 2. XQuartz Configuration
After installation, configure XQuartz:

1. Launch XQuartz:
```bash
open -a XQuartz
```

2. Configure settings:
   - Open XQuartz Preferences (⌘,)
   - Go to the "Security" tab
   - Enable "Allow connections from network clients"
   - Check "Authenticate connections"
   - Restart XQuartz for changes to take effect

#### 3. Verify XQuartz Setup
```bash
# Check if XQuartz is running
ps aux | grep XQuartz

# Check X11 forwarding
echo $DISPLAY
```

## Building the Docker Image

### Linux Build
```bash
# Change directory to scripts
cd scripts/
# Run the build script
./build_linux.sh
```

The Linux build:
- Uses standard X11 forwarding
- Includes NVIDIA GPU support
- Optimized for native Linux graphics performance

### MacOS Build
```bash
# Change directory to scripts
cd scripts/
# Run the build script
./build_mac.sh
```

The MacOS build:
- Includes X11 forwarding configuration for XQuartz
- Adds necessary X11 client libraries
- Configures display settings for MacOS compatibility

## Running the Container

### Linux Run
```bash
# Change directory to scripts
cd scripts/
# Run the container
./run_linux.sh
```

The Linux run script:
- Enables GPU passthrough
- Sets up X11 display forwarding
- Configures network settings for ROS2
- Mounts the source directory

### MacOS Run
```bash
# Change directory to scripts
cd scripts/
# Run the container
./run_mac.sh
```

The MacOS run script:
- Starts XQuartz if not running
- Configures X11 display forwarding
- Sets up network settings for ROS2
- Mounts the source directory

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

1. XQuartz Connection Issues:
```bash
# Verify XQuartz is running
ps aux | grep XQuartz

# Reset XQuartz permissions
xhost + localhost
```

2. Display Problems:
```bash
# Check DISPLAY variable
echo $DISPLAY

# Should show: host.docker.internal:0
```

3. Slow Graphics Performance:
- Try adjusting indirect rendering:
```bash
export LIBGL_ALWAYS_INDIRECT=1
```

### Testing GUI Applications

After starting the container, test the GUI setup:
```bash
# Simple X11 test
xeyes

# Test RViz
ros2 run rviz2 rviz2

# Test Gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

If any GUI application fails to start:
1. Check the DISPLAY environment variable
2. Verify X11 forwarding is working
3. Ensure all required ROS2 packages are installed
4. Check system logs for error messages

## Notes
- Keep the source directory updated
- Rebuild the container if Dockerfile changes
- For MacOS users: XQuartz must be running before starting the container
- For Linux users: Ensure NVIDIA drivers are up to date if using GPU
