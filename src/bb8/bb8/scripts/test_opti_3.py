# Initial test of differential drive
import casadi as cas
import rclpy
import time
import numpy as np

from bb8.scripts.test_command import BB8Tests
from bb8.scripts.test_diffdrive_init import cas_command 

def main(args=None):
    rclpy.init(args=args)

    # Optimization
    N = 100  #number of control intervals

    cmd = cas_command(N)
    cmd.opti.subject_to(cmd.X_1[0,cmd.N-1]==0.1) # Final x position in meters

    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt')
    sol = cmd.opti.solve()

    # Get optimal solutions
    cmd.V_R_solution = sol.value(cmd.V_R)
    cmd.V_L_solution = sol.value(cmd.V_L)
    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.T_solution = sol.value(cmd.T)

    # Set commands based on optimal solution
    cmd.linear_x = np.insert(cmd.X_1_solution[0, 0:N-1] - cmd.X_1_solution[0, 1:N], 0, 0)
    cmd.linear_y = np.insert(cmd.X_1_solution[1, 0:N-1] - cmd.X_1_solution[1, 1:N], 0, 0)
    cmd.angular_z = np.insert(cmd.X_1_solution[2, 0:N-1] - cmd.X_1_solution[2, 1:N], 0, 0)

    # Initialize commands
    bb8_tests_obj = BB8Tests()

    # Send commands
    bb8_tests_obj.clear_commands()
    time.sleep(1.0) # wait 1 second
    bb8_tests_obj.test_commands(cmd)
    time.sleep(1.0) # wait 1 second
    bb8_tests_obj.clear_commands()

    # Starts Node
    rclpy.spin(bb8_tests_obj)
    
    # Shutdown and clean up
    bb8_tests_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()