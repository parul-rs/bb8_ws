import casadi as cas
import rclpy
import time

from bb8.scripts.test_init import cas_command
from bb8.scripts.test_command import BB8Tests

def main(args=None):
    rclpy.init(args=args)
    bb8_tests_obj = BB8Tests()
    
    # Optimization

    N = 100  # number of control intervals
    alpha_list = [0]*(N-1)

    cmd = cas_command(N, alpha_list)
    cmd.opti.subject_to(cmd.X_1[0,cmd.N-1]==6.28318) # Final angular position
    cmd.opti.subject_to(cmd.X_2[0,cmd.N-1]==6.28318) # Final angular position 1

    # Solve using ipop`t solver for nonlinear optimization problem
    cmd.opti.solver('ipopt')
    sol = cmd.opti.solve()

    cmd.X_1_solution = sol.value(cmd.X_1)
    cmd.X_2_solution = sol.value(cmd.X_2)
    cmd.T_solution = sol.value(cmd.T)

    cmd.linear_x = cmd.X_2_solution[0, :] * cmd.r
    cmd.angular_z = cmd.X_1_solution[1, :]

    # bb8_tests_obj.clear_commands(cmd)
    bb8_tests_obj.clear_commands()
    time.sleep(1.0) # wait 1 second
    bb8_tests_obj.test_commands(cmd)
    time.sleep(1.0) # wait 1 second
    bb8_tests_obj.clear_commands()
    rclpy.spin(bb8_tests_obj)
    
    # Shutdown and clean up
    bb8_tests_obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()