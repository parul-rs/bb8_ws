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
        dt = cmd.T_solution/cmd.N

        for sim_index in range(cmd.N):
            print(sim_index)
            if not rclpy.ok():
                break
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = float(cmd.linear_x[sim_index])
            cmd_vel_msg.linear.y = float(cmd.linear_y[sim_index])
            cmd_vel_msg.linear.z = float(cmd.linear_z[sim_index])
            cmd_vel_msg.angular.x = float(cmd.angular_x[sim_index])
            cmd_vel_msg.angular.y = float(cmd.angular_y[sim_index])
            cmd_vel_msg.angular.z = float(cmd.angular_z[sim_index])
            self.publisher.publish(cmd_vel_msg)
            time.sleep(dt)

def main(args=None):
    rclpy.init(args=args)
    
    bb8_tests_obj = BB8Tests()
    
    class CmdStraight:
        T_solution = 5  # Time to move straight
        N = 50  # Number of commands
        linear_x = [0.5] * N  # Move forward with 0.5 m/s
        linear_y = [0.0] * N
        linear_z = [0.0] * N
        angular_x = [0.0] * N
        angular_y = [0.0] * N
        angular_z = [0.0] * N  # No rotation
    
    class CmdTurn90:
        T_solution = 2  # Total duration of the turn in seconds
        N = 20  # Number of commands (2 seconds at 10 Hz)
        linear_x = [0.0] * N  # No forward motion during the turn
        linear_y = [0.0] * N
        linear_z = [0.0] * N
        angular_x = [0.0] * N
        angular_y = [0.0] * N
        angular_z = [0.785] * N  # Angular velocity for 90-degree turn in 2 seconds

    try:
        # Drive straight
        print("Driving straight...")
        bb8_tests_obj.test_commands(CmdStraight)
        
        # Turn (choose CmdTurnLeft or CmdTurnRight based on desired direction)
        print("Turning left...")
        bb8_tests_obj.test_commands(CmdTurn90)  # Replace with CmdTurnRight to turn right
        
        # Drive straight again
        print("Driving straight again...")
        bb8_tests_obj.test_commands(CmdStraight)
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Clear commands and clean up
        bb8_tests_obj.clear_commands()
        bb8_tests_obj.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()