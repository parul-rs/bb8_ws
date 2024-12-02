# Initial test of differential drive
import casadi as cas
import rclpy
import time
import numpy as np

from bb8.scripts.test_command import BB8Tests
from bb8.scripts.test_diffdrive_init import cas_command 

def main(args=None):
    rclpy.init(args=args)

    points = [
        [3,3],
        [0,3],
        [0,0],
        [3,0],
        [3,1],
        [2,1],
        [4,0],
        [4,3],
        [7,3],
        [7,1],
        [4,1],
        [5,1],
        [6,0],
        [8,0],
        [8,3],
        [11,3],
        [11,0],
        [8,0],  
        [12,0],
        [12,3],
        [15,3],
        [15,0],
        [12,0],
        [17.5,0],
        [17.5,3],
        [16,3],
        [19,3],
    ]

    p_array = np.array(points)

    # Optimization
    N = 50  #number of control intervals

    cmd = cas_command(N)
    # X_1[0,:] = x pos
    # X_1[1,:] = y pos
    # X_1[2,:] = rotation angle
    
    for n in range(p_array.shape[0]):
        cmd.opti.subject_to(cas.fabs(cmd.X_1[0,n]-p_array[n,0]) <= 0.1) # x pos at n
        cmd.opti.subject_to(cas.fabs(cmd.X_1[1,n]-p_array[n,1]) <= 0.1) # y pos at n

    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt')
    try:
        sol = cmd.opti.solve()
    except:
        cmd.opti.debug.value

    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.T_solution = N*cmd.dt
    cmd.V_R_solution = sol.value(cmd.V_R)
    cmd.V_L_solution = sol.value(cmd.V_L)

    # Set commands based on optimal solution
    cf_x_lin_vel = np.array((cmd.X_1_solution[0, 0:cmd.N-1] - cmd.X_1_solution[0, 1:N])/cmd.dt)
    cf_y_lin_vel = np.array((cmd.X_1_solution[1, 0:cmd.N-1] - cmd.X_1_solution[1, 1:N])/cmd.dt)
    cmd.linear_x = cf_x_lin_vel*np.cos(cmd.X_1_solution[2, 0:cmd.N-1]) - cf_y_lin_vel*np.sin(cmd.X_1_solution[2, 0:cmd.N-1])
    cmd.linear_y = cf_x_lin_vel*np.sin(cmd.X_1_solution[2, 0:cmd.N-1]) + cf_y_lin_vel*np.cos(cmd.X_1_solution[2, 0:cmd.N-1])
    cmd.angular_z = np.array((cmd.X_1_solution[2, 0:cmd.N-1] - cmd.X_1_solution[2, 1:N])/cmd.dt)
    cmd.N = cmd.N - 1

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