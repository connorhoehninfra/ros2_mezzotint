#!/bin/bash
docker build --build-arg ON_MAC=false -f ../Dockerfile -t ros2_jazzy_my_cobot:latest ..