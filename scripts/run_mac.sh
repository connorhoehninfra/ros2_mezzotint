#!/bin/bash

# Ensure DISPLAY is set
export DISPLAY=:0
echo "DISPLAY is set to $DISPLAY"

# Allow X11 forwarding
xhost +localhost

# Run the ROS2 Docker container with GUI support
echo "Starting ROS2 Jazzy container..."
docker run -it --rm \
    --name ros2_jazzy_my_cobot \
    -v "$(pwd)/src:/root/colcon_ws/src" \
    -e DISPLAY=$DISPLAY \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    -e MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
    -e MESA_GL_VERSION_OVERRIDE=3.3 \
    -e MESA_GLSL_VERSION_OVERRIDE=330 \
    --network host \
    ros2_jazzy_my_cobot:mac /bin/bash -c "
        source /opt/ros/jazzy/setup.bash &&
        export DISPLAY=$DISPLAY &&
        glxinfo | grep 'OpenGL version' &&
        exec /bin/bash"

