#! /usr/bin/env python

import rclpy # EDIT : from rospy (ros1)
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class BB8Tests(Node):

    def __init__(self):
        super().__init__('test_command_node') # Initialize ROS2 Node 
        # Create publisher to send command veleocities to /cmd_vel topic 
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def clear_commands(self):
        """
        This clears commands to linear vel x, y, z and angular vel x, y, z
        :return:
        """
                
        cmd_vel_msg = Twist()
        self.publisher.publish(cmd_vel_msg)

    def test_commands(self, cmd):
        """
        This passes commands to linear vel x, y, z and angular vel x, y, z
        :return:
        """

        # command delta time 1/Hz
        dt = cmd.T/cmd.N

        for sim_index in range(cmd.N):
            if not rclpy.ok():
                break
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = cmd.linear_x[sim_index]
            cmd_vel_msg.linear.y = cmd.linear_y[sim_index]
            cmd_vel_msg.linear.z = cmd.linear_z[sim_index]
            cmd_vel_msg.angular.x = cmd.angular_x[sim_index]
            cmd_vel_msg.angular.y = cmd.angular_y[sim_index]
            cmd_vel_msg.angular.z = cmd.angular_z[sim_index]
            self.publisher.publish(cmd_vel_msg)
            time.sleep(dt)

def main(args=None):
    rclpy.init(args=args)
    
    bb8_tests_obj = BB8Tests()
    
    # Example command input
    class Cmd:
        T = 10  # Total time
        N = 100  # Number of commands
        linear_x = [0.1] * 100  # Example linear velocities
        linear_y = [0.0] * 100  # Example linear velocities
        linear_z = [0.0] * 100
        angular_x = [0.0] * 100
        angular_y = [0.0] * 100
        angular_z = [0.1] * 100  # Example angular velocities

    bb8_tests_obj.test_commands(Cmd)
    rclpy.spin(bb8_tests_obj)
    
    # Shutdown and clean up
    bb8_tests_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
