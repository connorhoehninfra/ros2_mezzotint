#!/bin/bash
docker build --platform=linux/arm64 --build-arg ON_MAC=true -f ../Dockerfile -t ros2_jazzy_my_cobot:mac ..
