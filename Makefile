# Start the container
start-container:
	docker compose up -d

# Connect to the container
connect-container:
	@if [ $$(docker ps -q -f name=ros2_desktop_vnc) ]; then \
		docker exec -it ros2_desktop_vnc /bin/bash -c "cd /home/ubuntu/Projects/ros2_mezzotint && \
		source /opt/ros/jazzy/setup.bash && \
		source install/setup.bash && \
		bash"; \
	else \
		echo "Container ros2_desktop_vnc is not running."; \
	fi


# Rebuild packages in the container
rebuild-packages:
	@if [ $$(docker ps -q -f name=ros2_desktop_vnc) ]; then \
		docker exec -it ros2_desktop_vnc /bin/bash -c "cd /home/ubuntu/Projects/ros2_mezzotint && \
		source /opt/ros/jazzy/setup.bash && \
		source install/setup.bash && \
		rm -rf build install log && \
		colcon build --symlink-install"; \
	else \
		echo "Container ros2_desktop_vnc is not running."; \
	fi

# Stop the container
stop-container:
	docker compose down

# Rebuild the container
rebuild-container: stop-container
	docker compose up --build -d
