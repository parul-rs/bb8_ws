import casadi as cas
import rospy
import time

from test_init import cas_command 
from test_command import BB8Tests

# Optimization

N = 100  #number of control intervals
alpha = 0

cmd = cas_command(N, alpha)
cmd.opti.subject_to(cmd.X_1[0,cmd.N-1]==6.28318) # Final angular position
cmd.opti.subject_to(cmd.X_2[0,cmd.N-1]==6.28318) # Final angular position 1

# Solve using ipop`t solver for nonlinear optimization problem
cmd.opti.solver('ipopt')
sol = cmd.opti.solve()

cmd.linear_x = cmd.X_2_solution[0, :] * cmd.r
cmd.angular_z = cmd.X_1_solution[1, :]

rospy.init_node('bb8_command_test_node', log_level=rospy.DEBUG)
bb8_tests_obj = BB8Tests()

bb8_tests_obj.clear_commands(cmd)
time.sleep(1.0) # wait 1 second
bb8_tests_obj.test_commands(cmd)
time.sleep(1.0) # wait 1 second
bb8_tests_obj.clear_commands(cmd)