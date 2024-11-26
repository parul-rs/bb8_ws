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
    N = 65  #number of control intervals

    cmd = cas_command(N)
    # X_1[0] = x pos
    # X_1[1] = y pos
    # X_1[2] = rotation angle
    
    # Instructing BB8 to go to edge of circle first
    cmd.opti.subject_to(cmd.X_1[0,4]== 20 )
    cmd.opti.subject_to(cmd.X_1[1,4]== 0 )
    cmd.opti.subject_to(cmd.X_1[2,4]== 0 )

    for n in range(0,N-5):
        cmd.opti.subject_to(cmd.X_1[0,n+5]== 20*np.cos(6*n*np.pi/180) ) # x pos at n
        cmd.opti.subject_to(cmd.X_1[1,n+5]== 20*np.sin(6*n*np.pi/180) ) # y pos at n
        cmd.opti.subject_to(cmd.X_1[2,n+5]== 6*n*np.pi/180 ) # rotation angle at n


    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt')
    sol = cmd.opti.solve()

    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.T_solution = N

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
