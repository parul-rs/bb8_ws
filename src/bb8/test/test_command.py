#! /usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import Twist
import time

class BB8Tests(object):

    def __init__(self):
        self._model = GetModelStateRequest()
        self._model.model_name = 'bb_8'
        rospy.wait_for_service('/gazebo/get_model_state')
        self._get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def clear_commands(self):
        """
        This clears commands to linear vel x, y, z and angular vel x, y, z
        :return:
        """
                
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = 0.0
        self._pub.publish(cmd_vel_msg)

    def test_commands(self, cmd):
        """
        This passes commands to linear vel x, y, z and angular vel x, y, z
        :return:
        """

        # command delta time 1/Hz
        dt = cmd.T_solution/cmd.N

        if not rospy.is_shutdown():
            for sim_index in range(cmd.N):
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = cmd.linear_x[sim_index]
                cmd_vel_msg.linear.y = cmd.linear_y[sim_index]
                cmd_vel_msg.linear.z = cmd.linear_z[sim_index]
                cmd_vel_msg.angular.x = cmd.angular_x[sim_index]
                cmd_vel_msg.angular.y = cmd.angular_y[sim_index]
                cmd_vel_msg.angular.z = cmd.angular_z[sim_index]
                self._pub.publish(cmd_vel_msg)
                time.sleep(dt)

if __name__ == "__main__":

    rospy.init_node('bb8_command_test_node', log_level=rospy.DEBUG)
    bb8_tests_obj = BB8Tests()
    bb8_tests_obj.test_commands()
