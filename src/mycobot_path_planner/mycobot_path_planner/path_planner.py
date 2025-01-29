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
            start_x, start_y, start_z = start_point
            end_x, end_y, end_z = end_point
            
            # Determine boundaries and directions
            x_min, x_max = sorted([start_x, end_x])
            y_dir = 1 if end_y > start_y else -1
            y_step = pitch_distance * y_dir

            # Generate y positions with precise endpoint
            y_positions = []
            current_y = start_y
            while (y_dir == 1 and current_y <= end_y) or (y_dir == -1 and current_y >= end_y):
                y_positions.append(current_y)
                if current_y == end_y:
                    break
                next_y = current_y + y_step
                if (y_dir == 1 and next_y > end_y) or (y_dir == -1 and next_y < end_y):
                    y_positions.append(end_y)
                    break
                current_y = next_y

            # Determine initial direction based on start position
            initial_right = start_x == x_min
            current_dir = initial_right
            path = []

            # Generate path points
            for y in y_positions:
                x_start = x_min if current_dir else x_max
                x_end = x_max if current_dir else x_min
                step = resolution if current_dir else -resolution

                current_x = x_start
                while (current_dir and current_x <= x_max) or (not current_dir and current_x >= x_min):
                    path.append((round(current_x, 8), round(y, 8), round(start_z, 8)))
                    if current_x == x_end:
                        break
                    next_x = current_x + step
                    if (current_dir and next_x > x_max) or (not current_dir and next_x < x_min):
                        path.append((round(x_end, 8), round(y, 8), round(start_z, 8)))
                        break
                    current_x = next_x
                
                current_dir = not current_dir  # Reverse direction for next line

            # Ensure exact endpoint match
            if path[-1] != end_point:
                path.append(end_point)

            self.get_logger().info(f"Zigzag path generation complete. Generated {len(path)} points")
            return path
            
        except Exception as e:
            self.get_logger().error(f"Error in zigzag path generation: {str(e)}, Generated a simple direct path as a fallback")
            # Return a simple direct path as fallback
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