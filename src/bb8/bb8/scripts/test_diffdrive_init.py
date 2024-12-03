import casadi as cas

class cas_command:

    def __init__(self, N, dt):
        # For reference, from journal paper: 'The total diameter of the robot is 18 centimetres'. This does not seem to match the dimensions in the thesis.
        # NOTE: NEED TO UPDATE VALUES ON XACRO FILE TO MATCH THESIS VALUES!!

        self.N = N

        # ---------------------------------------------------------
        # Spherical robot's properties
        self.r = 0.35 # radius of wheel in meters
        self.L = 0.70 # distance between wheels in meters
        
        # -------------------------------------------------------------------------------------------
        # Defining EOM
        self.theta = cas.SX.sym("theta") # heading (rad)
        self.v_r = cas.SX.sym("v_r") # velocity of right wheel
        self.v_l = cas.SX.sym("v_l") # velocity of left wheel
        self.x = cas.SX.sym("x") # x axis translation (m)
        self.y = cas.SX.sym("y") # y axis translation (m)

        # Define ODE
        self.x_1 = cas.vertcat(self.x,self.y,self.theta) # state
        self.ode_1 = cas.vertcat(
            self.r * (self.v_r + self.v_l)/2 * cas.cos(self.theta),
            self.r * (self.v_r + self.v_l)/2 * cas.sin(self.theta),
            self.r / self.L * (self.v_r - self.v_l)
            ) # system of ODEs
        # -------------------------------------------------------------------------------------------

        self.opti = cas.Opti() # Optimization problem

        self.X_1 = self.opti.variable(3,N) # state trajectory
        self.V_R = self.opti.variable(1,N) # control trajectory - Right Speed
        self.V_L = self.opti.variable(1,N) # control trajectory - Left Speed

        self.opti.minimize(cas.sumsqr(self.V_R[:]) + cas.sumsqr(self.V_L[:])) # reach the goal using least motor command
  
        self.ode_function_1 = cas.Function('ode_function', [self.x_1, self.v_r, self.v_l], [self.ode_1])

        self.dt = dt

        for i in range(N-1):
            # Foward Euler Method
            x_1_dot_i = self.ode_function_1(self.X_1[:,i], self.V_R[i], self.V_L[i])
            x_1_next = self.X_1[:,i] + self.dt * x_1_dot_i
            self.opti.subject_to(self.X_1[:,i+1]==x_1_next) # Close the integration gaps for x_1
        
        self.linear_x = [0] * self.N
        self.linear_y = [0] * self.N
        self.linear_z = [0] * self.N
        self.angular_x = [0] * self.N
        self.angular_y = [0] * self.N
        self.angular_z = [0] * self.N