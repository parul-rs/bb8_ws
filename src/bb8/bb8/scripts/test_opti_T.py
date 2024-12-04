# Groot test of differential drive
import casadi as cas
import rclpy
import time
import numpy as np

from bb8.scripts.test_command import BB8Tests
from bb8.scripts.test_diffdrive_init import cas_command 

def main(args=None):
    rclpy.init(args=args)

    points = [
        [0,0],
        [0.5,0],
        [0.5,1],
        [0,1],
        [1,1],
    ]

    p_array = np.array(points)

    # Optimization
    N = (5+1)*3*5  #number of control intervals
    dt = 0.2
    cmd = cas_command(N,dt)
    # X_1[0,:] = x pos
    # X_1[1,:] = y pos
    # X_1[2,:] = rotation angle
    cmd.opti.subject_to(cmd.X_1[0,0] == 0) # Initial x Condition
    cmd.opti.subject_to(cmd.X_1[1,0] == 0) # Initial y Condition
    cmd.opti.subject_to(cmd.X_1[2,0] == 0) # Initial theta Condition
    cmd.opti.subject_to(cmd.V_R[0] == 0) # Initial V_R Condition
    cmd.opti.subject_to(cmd.V_L[0] == 0) # Initial V_L Condition

    # Define new xy position every 15 steps
    for n in range(p_array.shape[0]):
        cmd.opti.subject_to(cas.fabs(cmd.X_1[0,n*3*5]-p_array[n,0]) <= 0.1) # x pos at n
        cmd.opti.subject_to(cas.fabs(cmd.X_1[1,n*3*5]-p_array[n,1]) <= 0.1) # y pos at n

    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt',{"expand":True}, {"max_iter":100000})
   
    sol = cmd.opti.solve()

    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.V_R_solution = sol.value(cmd.V_R)
    cmd.V_L_solution = sol.value(cmd.V_L)

    # Set commands based on optimal solution
    cmd.linear_x = (cmd.V_R_solution + cmd.V_L_solution) * cmd.r / 2
    cmd.angular_z = (cmd.V_R_solution - cmd.V_L_solution) * cmd.r / cmd.L 

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