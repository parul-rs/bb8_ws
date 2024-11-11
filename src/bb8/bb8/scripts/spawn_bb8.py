#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class SpawnBB8(Node):
    def __init__(self):
        super().__init__('spawn_bb8')
        self.client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service to be available...')
        
        # Create a request to spawn the robot
        self.request = SpawnEntity.Request()
        self.request.name = 'bb8'
        self.request.xml = 'URDF_XML_CONTENT_HERE'  # Replace with the actual URDF or Xacro string

    def send_request(self):
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('Successfully spawned BB8')
        else:
            self.get_logger().error('Failed to spawn BB8')

def main(args=None):
    rclpy.init(args=args)
    node = SpawnBB8()
    node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
