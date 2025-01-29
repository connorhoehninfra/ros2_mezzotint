# Start the container
start-container:
	docker compose up -d

# Connect to the container
connect-container:
	@if [ $$(docker ps -q -f name=ros2_desktop_vnc) ]; then \
		docker exec -it ros2_desktop_vnc /bin/bash; \
	else \
		echo "Container ros2_desktop_vnc is not running."; \
	fi

# Placeholder for launching rviz2 inside the container
launch-rviz2:
	@if [ $$(docker ps -q -f name=ros2_desktop_vnc) ]; then \
		docker exec -d ros2_desktop_vnc rviz2; \
	else \
		echo "Container ros2_desktop_vnc is not running."; \
	fi

# Stop the container
stop-container:
	docker compose down

# Rebuild the container
rebuild-container: stop-container
	docker compose up --build -d
