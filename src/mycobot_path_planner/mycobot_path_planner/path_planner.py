import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from cobot_interfaces.srv import PlanPath
import traceback

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.plan_path_service = self.create_service(PlanPath, 'plan_path', self.plan_path_callback)
        self.path_pub          = self.create_publisher(MarkerArray, 'path_visualization_marker_array', 10)
        self.get_logger().info('Path Planner node initialized')
    
    # Plan path service callback 
    def plan_path_callback(self, request, response):
        try:
            self.get_logger().info("Received path planning request")
            
            # Validate input parameters
            if len(request.start_point) != 3 or len(request.end_point) != 3:
                self.get_logger().error("Invalid start or end point dimensions")
                response.success = False
                response.path_points = []
                return response
            elif request.resolution <= 0 or request.pitch_distance <= 0:
                self.get_logger().error("Invalid resolution or pitch distance")
                response.success = False
                response.path_points = []
                return response

            # Generate zigzag path using service request parameters
            path = self.generate_zigzag_path(
                start_point=list(request.start_point),
                end_point=list(request.end_point),
                resolution=request.resolution,
                pitch_distance=request.pitch_distance
            )
            self.get_logger().info(f"Generated path with {len(path)} points")
            
            # Convert path tuples to Point messages
            response.path_points = [Point(x=p[0], y=p[1], z=p[2]) for p in path]
            
            # Publish visualization
            try:
                self.publish_path_marker(path)
                self.get_logger().info("Published path markers")
                response.success = True
            except Exception as viz_error:
                self.get_logger().error(f"Failed to publish markers: {str(viz_error)}")
                self.get_logger().error(traceback.format_exc())
                response.success = False
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error in path planning callback: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            response.success = False
            response.path_points = []
            return response


    def generate_zigzag_path(self, start_point, end_point, resolution, pitch_distance):
        """
        Generate a zigzag path from start_point to end_point in the x-y plane.
        """
        try:
            self.get_logger().info(f"Starting zigzag path generation with params:")
            start_x, start_y, start_z = start_point[0], start_point[1], start_point[2]
            end_x, end_y, end_z = end_point[0], end_point[1], end_point[2]
            resolution = resolution
            pitch_distance = pitch_distance
            
            self.get_logger().info(f"Start: [{start_x}, {start_y}, {start_z}], End: [{end_x}, {end_y}, {end_z}]")
            self.get_logger().info(f"Resolution: {resolution}, Pitch: {pitch_distance}")
            
            path = []
            current_x, current_y, current_z = start_x, start_y, start_z
            
            # Direction flag: True for left-to-right, False for right-to-left
            going_right = True
            
            # Add the starting point
            path.append((round(current_x, 8), round(current_y, 8), round(current_z, 8)))
            
            # Safety counter to prevent infinite loops
            max_iterations = int((abs(start_y - end_y) / pitch_distance + 2) * 
                            (abs(start_x - end_x) / resolution + 2))
            iteration_count = 0
            
            while current_y > end_y - pitch_distance/2:
                iteration_count += 1
                if iteration_count > max_iterations:
                    self.get_logger().warn("Maximum iterations exceeded in zigzag generation")
                    break
                    
                if going_right:
                    # Move right
                    while current_x > end_x:
                        current_x -= resolution
                        current_x = max(current_x, end_x)
                        path.append((round(current_x, 8), round(current_y, 8), round(current_z, 8)))
                else:
                    # Move left
                    while current_x < start_x:
                        current_x += resolution
                        current_x = min(current_x, start_x)
                        path.append((round(current_x, 8), round(current_y, 8), round(current_z, 8)))

                # Move down if not at end_y
                if current_y > end_y:
                    current_y -= pitch_distance
                    current_y = max(current_y, end_y)
                    path.append((round(current_x, 8), round(current_y, 8), round(current_z, 8)))
                
                # Toggle direction
                going_right = not going_right
                
                # Check if reached the endpoint
                if current_x == end_x and current_y == end_y:
                    break
            
            # Ensure the last point is exactly the end point
            if path[-1] != (end_x, end_y, end_z):
                path.append((end_x, end_y, end_z))

            self.get_logger().info(f"Zigzag path generation complete. Generated {len(path)} points")
            return path
            
        except Exception as e:
            self.get_logger().error(f"Error in zigzag path generation: {str(e)}, Generated a simple direct path as a fallback")
            # Return a simple direct path as fallback
            return [(start_point[0], start_point[1], start_point[2]),
                    (end_point[0], end_point[1], end_point[2])]

    def generate_straight_line_path(self, start_point, end_point, resolution):
        """Generate a simple straight line path between two points for testing"""
        try:
            self.get_logger().info("Generating straight line path")
            path = []
            
            # Calculate total distance
            dx = end_point[0] - start_point[0]
            dy = end_point[1] - start_point[1]
            dz = end_point[2] - start_point[2]
            
            # Calculate number of points needed
            distance = (dx**2 + dy**2 + dz**2)**0.5
            num_points = max(int(distance/resolution), 2)  # Minimum 2 points
            
            self.get_logger().info(f"Distance: {distance}, Number of points: {num_points}")
            
            # Generate points along the line
            for i in range(num_points):
                t = i / (num_points - 1)
                x = start_point[0] + t * dx
                y = start_point[1] + t * dy
                z = start_point[2] + t * dz
                path.append((x, y, z))
            
            return path
            
        except Exception as e:
            self.get_logger().error(f"Error generating path: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            return [(start_point[0], start_point[1], start_point[2]), 
                   (end_point[0], end_point[1], end_point[2])]

    def create_marker(self, marker_type, ns, id, scale, color, frame_id="world", action=Marker.ADD):
        """
        Utility function to create a customizable Marker message.
        Parameters:
        - marker_type: Type of the marker (e.g., Marker.SPHERE, Marker.CYLINDER, etc.)
        - ns: Namespace of the marker.
        - id: Unique ID of the marker.
        - scale: Scale of the marker as a tuple (x, y, z).
        - color: Color of the marker as a tuple (r, g, b, a).
        - frame_id: Frame ID of the marker (default is "world").
        - action: Action for the marker (default is Marker.ADD).

        Returns:
        - marker: A configured Marker message.
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = id
        marker.type = marker_type
        marker.action = action
        marker.scale = scale
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker 

    def publish_path_marker(self, path):        
        marker_array = MarkerArray()
        path_marker = self.create_marker(
                marker_type=Marker.LINE_STRIP,
                ns="path_points",
                id=0,
                scale=Vector3(x=0.01, y=0.0, z=0.0),
                color=(0.0, 0.3, 0.7, 0.8)
            )
        for point in path:
            x, y, z = point
            point = Point(x=x, y=y, z=z)
            path_marker.points.append(point)
        marker_array.markers.append(path_marker)
        
       # Create start point marker
        if len(path) > 0:
            start_marker = self.create_marker(
                marker_type=Marker.SPHERE,
                ns="start_point",
                id=1,
                scale=Vector3(x=0.02, y=0.02, z=0.02),
                color=(0.0, 1.0, 0.0, 0.8)
            )
            start_marker.pose.position.x = path[0][0]
            start_marker.pose.position.y = path[0][1]
            start_marker.pose.position.z = path[0][2]
            marker_array.markers.append(start_marker)

        # Create end point marker
        if len(path) > 1:
            end_marker = self.create_marker(
                marker_type=Marker.SPHERE,
                ns="end_point",
                id=2,
                scale=Vector3(x=0.02, y=0.02, z=0.02),
                color=(1.0, 0.0, 0.0, 0.8)
            )
            end_marker.pose.position.x = path[-1][0]
            end_marker.pose.position.y = path[-1][1]
            end_marker.pose.position.z = path[-1][2]
            marker_array.markers.append(end_marker)
        self.path_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        path_planner = PathPlanner()
        rclpy.spin(path_planner)
    except Exception as e:
        print(f"Error in main: {str(e)}")
        traceback.print_exc()
    finally:
        if 'path_planner' in locals():
            path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()