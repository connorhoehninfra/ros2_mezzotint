#!/bin/bash
xhost +local:docker

docker run -it --rm --privileged --net=host --name ros2_jazzy_my_cobot \
    -v /home/zx/docker_ros2_jazzy/src:/root/colcon_ws/src \
    --env=NVIDIA_DRIVER_CAPABILITIES=all \
    --env=QT_X11_NO_MITSHM=1 \
    --env=DISPLAY \
    --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=0 \
    ros2_jazzy_my_cobot:latest /bin/bash -c "source /opt/ros/jazzy/setup.bash && /bin/bash" 