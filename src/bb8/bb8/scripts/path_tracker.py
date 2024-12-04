import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import SpawnEntity
import math

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        
        # Robot's initial position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_spawn_x = 0.0
        self.last_spawn_y = 0.0
        self.spawn_threshold = 0.15
        
        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.update_position, 10)
        
        # Publisher for markers
        self.marker_pub = self.create_publisher(Marker, '/robot_path', 10)
        
        # Timer to periodically publish markers
        self.timer = self.create_timer(0.1, self.publish_marker)
        
        # Time tracking
        self.last_time = self.get_clock().now()

        # Marker initialization
        self.marker = Marker()
        self.marker.header.frame_id = "world"
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.2  # Sphere size
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0  # Red 
        self.marker.points = []

        # Create a service client for spawn_entity
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Spawn request message
        self.spawn_request = SpawnEntity.Request()
        self.spawn_request.name = "sphere"  # Correct attribute for model name
        #self.spawn_request.xml = self.generate_sphere_model_xml()  # Model URDF

    def update_position(self, msg: Twist):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Extract linear and angular velocity
        v = msg.linear.x
        omega = msg.angular.z
        
        # Print velocities for debugging
        print(f"Linear Velocity: {v}, Angular Velocity: {omega}")

        # Update position using differential drive equations
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # Normalize theta to [0, 2*pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Print updated position
        print(f"Updated Position: ({self.x}, {self.y}), Orientation: {self.theta}")

        # Check if the robot has moved enough to spawn a new sphere
        distance = math.sqrt((self.x - self.last_spawn_x)**2 + (self.y - self.last_spawn_y)**2)
        if distance > self.spawn_threshold:
            self.last_spawn_x = self.x
            self.last_spawn_y = self.y
            self.spawn_sphere(self.x, self.y)


    def publish_marker(self):
        # Add current position to the marker points
        point = Point()
        point.x = self.x
        point.y = self.y
        point.z = 0.0  # Keep the z-coordinate at 0 (ground level)
        
        # Append the point to the marker
        self.marker.points.append(point)
        
        # Update the marker header with the current timestamp
        self.marker.header.stamp = self.get_clock().now().to_msg()
        
        # Publish the marker
        self.marker_pub.publish(self.marker)


    def spawn_sphere(self, x, y):
        # Calculate the position behind the robot
        distance_behind = 1.0  # Distance to spawn the sphere behind the robot (adjust as needed)
        
        # Calculate the offset based on the robot's orientation
        offset_x = distance_behind * math.cos(self.theta + math.pi)  # Add math.pi to get the opposite direction
        offset_y = distance_behind * math.sin(self.theta + math.pi)  # Same for the y-direction

        # New position for the sphere behind the robot
        spawn_x = x + offset_x
        spawn_y = y + offset_y

        # Update the last spawn positions
        self.last_spawn_x = spawn_x
        self.last_spawn_y = spawn_y

        # Generate a unique name for the sphere based on the updated spawn position
        sphere_name = f"sphere_{self.last_spawn_x:.2f}_{self.last_spawn_y:.2f}"  # Using formatted string for better precision
        self.get_logger().info(f"Spawning sphere behind the robot at: ({spawn_x}, {spawn_y}) with name: {sphere_name}")
        
        # Update the spawn request with the new position
        self.spawn_request.name = sphere_name
        self.spawn_request.initial_pose.position.x = spawn_x
        self.spawn_request.initial_pose.position.y = spawn_y
        self.spawn_request.initial_pose.position.z = 0.0  # Keep Z at 0 (on the ground)

        # Dynamically generate the URDF with the current position
        self.spawn_request.xml = self.generate_sphere_model_xml(sphere_name, spawn_x, spawn_y)

        # Call the spawn entity service
        self.spawn_client.call_async(self.spawn_request)


    def generate_sphere_model_xml(self, x, y, sphere_name):
        # URDF for a simple sphere model with Gazebo-specific tags, dynamically adding the position
        sphere_urdf = f"""
        <robot name="{sphere_name}">
            <link name="sphere_link">
                <!-- Inertial properties of the sphere -->
                <inertial>
                    <origin rpy="0 0 0" xyz="0 0 0"/> <!-- This is the position of the origin relative to the link -->
                    <mass value="80.0"/>
                    <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
                </inertial>

                <!-- Collision properties of the sphere -->
                <collision>
                    <origin rpy="0 0 0" xyz="0 0 0"/> <!-- Collision origin relative to the link -->
                    <geometry>
                        <sphere radius="0.05"/>
                    </geometry>
                </collision>

                <!-- Visual properties of the sphere -->
                <visual>
                    <origin rpy="0 0 0" xyz="0 0 0"/> <!-- Visual origin relative to the link -->
                    <geometry>
                        <sphere radius="0.05"/>
                    </geometry>
                    <material name="RedMaterial"/>
                </visual>
            </link>

            <!-- Spawning at the robot's dynamic position -->
            <gazebo>
                <spawn x="{x}" y="{y}" z="0.0"/>
            </gazebo>
            
        </robot>
        """
        return sphere_urdf



def main(args=None):
    rclpy.init(args=args)
    path_tracker_node = PathTracker()
    rclpy.spin(path_tracker_node)

    # Cleanup
    path_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


