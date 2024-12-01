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
    N = 50  #number of control intervals

    cmd = cas_command(N)
    cmd.opti.subject_to(cmd.X_1[:,0]==[0,0,0]) # Initial condition
    cmd.opti.subject_to(cmd.V_R[0,0]==0) # Initial condition
    cmd.opti.subject_to(cmd.V_L[0,0]==0) # Initial condition
    cmd.opti.subject_to(cmd.X_1[0,np.floor(cmd.N / 2)]==1) # Mid x position in meters
    cmd.opti.subject_to(cmd.X_1[0,cmd.N-1]==0.0) # Final x position in meters

    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt')
    sol = cmd.opti.solve()

    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.V_R_solution = sol.value(cmd.V_R)
    cmd.V_L_solution = sol.value(cmd.V_L)

    # Set commands based on optimal solution
    cf_x_lin_vel = np.array((cmd.X_1_solution[0, 0:cmd.N-1] - cmd.X_1_solution[0, 1:N])/cmd.dt)
    cf_y_lin_vel = np.array((cmd.X_1_solution[1, 0:cmd.N-1] - cmd.X_1_solution[1, 1:N])/cmd.dt)
    cmd.linear_x = cf_x_lin_vel*np.cos(cmd.X_1_solution[2, 0:cmd.N-1]) - cf_y_lin_vel*np.sin(cmd.X_1_solution[2, 0:cmd.N-1])
    cmd.linear_y = cf_x_lin_vel*np.sin(cmd.X_1_solution[2, 0:cmd.N-1]) + cf_y_lin_vel*np.cos(cmd.X_1_solution[2, 0:cmd.N-1])
    cmd.angular_z = np.array((cmd.X_1_solution[2, 0:cmd.N-1] - cmd.X_1_solution[2, 1:N])/cmd.dt)

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