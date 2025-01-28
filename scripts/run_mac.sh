#!/bin/bash

# Start XQuartz if not running
if ! ps aux | grep XQuartz | grep -v grep > /dev/null; then
    open -a XQuartz
fi

# Wait for XQuartz to start
sleep 5

# Allow connections from localhost
xhost + localhost

docker run -it --rm \
    --name ros2_jazzy_my_cobot \
    -v "$(pwd)/src:/root/colcon_ws/src" \
    -e DISPLAY=host.docker.internal:0 \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    --network host \
    ros2_jazzy_my_cobot:mac /bin/bash -c "source /opt/ros/jazzy/setup.bash && /bin/bash"