#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class SinusoidalFloorMover(Node):

    def __init__(self):
        super().__init__('world_wiggler')
        
        # Create a client for the SetModelState service
        self.client = self.create_client(SetModelState, '/gazebo/set_model_state')
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /gazebo/set_model_state...')
        
        # Create the ModelState message for the floor
        self.floor_state = ModelState()
        self.floor_state.model_name = "floor"
        self.floor_state.pose.position.x = 0.0
        self.floor_state.pose.position.y = 0.0
        self.floor_state.pose.position.z = 0.05  # Initial height of the floor
        
        # Amplitude and frequency for the sinusoidal movement
        self.amplitude = 0.1  # meters
        self.frequency = 0.2  # Hz
        
        # Set a timer to periodically update the floor's position
        self.timer = self.create_timer(1.0 / 30.0, self.update_floor_position)  # 30 Hz

        self.get_logger().info("Sinusoidal floor mover started!")

    def update_floor_position(self):
        # Get the current time
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Update the floor's y-position with a sinusoidal function
        self.floor_state.pose.position.y = self.amplitude * math.sin(2 * math.pi * self.frequency * current_time)

        # Send the updated position to Gazebo
        request = SetModelState.Request()
        request.model_state = self.floor_state
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Floor state updated successfully')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SinusoidalFloorMover()

    # Spin to keep the node running
    rclpy.spin(node)

    # Shutdown ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
