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
    N = 200  #number of control intervals

    cmd = cas_command(N)
    # X_1[0,:] = x pos
    # X_1[1,:] = y pos
    # X_1[2,:] = rotation angle
    
    # Instructing BB8 to go to edge of circle first
    cmd.opti.subject_to(cmd.X_1[0,0] == 0) # Initial x Condition
    cmd.opti.subject_to(cmd.X_1[1,0] == 0) # Initial y Condition
    cmd.opti.subject_to(cmd.X_1[2,0] == 0) # Initial theta Condition

    for n in range(1,N-2,5):
        cmd.opti.subject_to(cas.fabs(cmd.X_1[0,n]-(4*cas.cos(n/(cmd.N-2)*2*np.pi-np.pi/2))) <= 0.1) # x pos at n
        cmd.opti.subject_to(cas.fabs(cmd.X_1[1,n]-(4*cas.sin(n/(cmd.N-2)*2*np.pi-np.pi/2)+4)) <= 0.1) # y pos at n

    cmd.opti.subject_to(cas.fabs(cmd.X_1[0,cmd.N-1]) <= 0.1) # Final x Condition
    cmd.opti.subject_to(cas.fabs(cmd.X_1[1,cmd.N-1]) <= 0.1) # Final y Condition
    cmd.opti.subject_to(cas.fabs(cmd.X_1[2,cmd.N-1]) <= 0.1) # Final theta Condition

    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt',{"expand":True}, {"max_iter":100000})
   
    sol = cmd.opti.solve()

    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.V_R_solution = sol.value(cmd.V_R)
    cmd.V_L_solution = sol.value(cmd.V_L)

    # Set commands based on optimal solution
    cmd.linear_x = (cmd.V_R_solution + cmd.V_L_solution) * cmd.r / 2 
    cmd.angular_z = (cmd.V_R_solution - cmd.V_L_solution) * cmd.r / cmd.L / 2

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
