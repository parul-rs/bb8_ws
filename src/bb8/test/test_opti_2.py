# Continuation of Test 1, introduces alpha (angle with surface) term in solving ODEs

import casadi as cas
import rospy
import time
import numpy as np

from test_pendulum_init import cas_command 
from test_command import BB8Tests

# Optimization

N = 100  #number of control intervals
n = int(N/2)
C = np.linspace(0, np.pi/2, n)
alpha_list = np.append(C, -C[::-1])

cmd = cas_command(N, alpha_list)
cmd.opti.subject_to(cmd.X_1[0,cmd.N-1]==6.28318) # Final angular position of spinning mode
cmd.opti.subject_to(cmd.X_2[0,cmd.N-1]==6.28318) # Final angular position of rolling mode

# Solve using ipop`t solver for nonlinear optimization problem
cmd.opti.solver('ipopt')
sol = cmd.opti.solve()

# Set commands based on optimal solution
cmd.linear_x = cmd.X_2_solution[0, :] * cmd.r
cmd.angular_z = cmd.X_1_solution[1, :]

# Initialize commands
rospy.init_node('bb8_command_test_node', log_level=rospy.DEBUG)
bb8_tests_obj = BB8Tests()

# Send commands
bb8_tests_obj.clear_commands(cmd)
time.sleep(1.0) # wait 1 second
bb8_tests_obj.test_commands(cmd)
time.sleep(1.0) # wait 1 second
bb8_tests_obj.clear_commands(cmd)