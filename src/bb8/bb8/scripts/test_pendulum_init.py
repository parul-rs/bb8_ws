import casadi as cas
import numpy as np

class cas_command:

    def __init__(self, N, alpha_list):
        # For reference, from journal paper: 'The total diameter of the robot is 18 centimetres'. This does not seem to match the dimensions in the thesis.
        # NOTE: NEED TO UPDATE VALUES ON XACRO FILE TO MATCH THESIS VALUES!!

        self.N = N
        self.alpha_list = alpha_list

        # ---------------------------------------------------------
        # Spherical robot's properties from master's thesis
        pend_mass = 1.795 # units: kg // symb: M2
        pend_inertia = 0.007428 # units: kg m^2 // symb: J2
        pend_inertia_shaft = 0.007728 # units: kg m^2 // symb: J2s
        dist_ball_cent_to_pend_CG = 0.065 # units: m // symb: e

        # Pedulum's properties from master's thesis
        sphere_mass = 3.294 # units: kg // symb: M1
        sphere_inertia = 0.0633 # units: kg m^2 // symb: J1
        sphere_inertia_shaft = 0.0777 # units: kg m^2 // symb: J1s
        sphere_radius_thesis = 0.226 # units: m // symb: r
        # ---------------------------------------------------------

        # -------------------------------------------------------------------------------------------
        # Changing variable names to match in the equations below
        self.J_1 = sphere_inertia
        self.J_2 = pend_inertia
        self.e = dist_ball_cent_to_pend_CG
        self.r = sphere_radius_thesis
        self.M_1 = sphere_mass
        self.M_2 = pend_mass
        # OTHER CONSTANTS
        self.B_1 = 0.1 # Units: kg * m^2 /s
        self.B_2 = 0.1 # Units: kg * m^2 /s
        self.J_eq = (1 / self.J_1) + (1 / self.J_2) # Equivalent moment of inertia
        # -------------------------------------------------------------------------------------------

        # -------------------------------------------------------------------------------------------
        # Defining EoM for turning
        '''
        From Modeling and Control of a Spherical Robot in the CoppeliaSim Simulator
        '''
        # Variables
        self.theta = cas.SX.sym("theta") # Angle of spherical robot casing
        self.omega = cas.SX.sym("omega") # Angular velocity of spherical robot
        self.omega_dot = cas.SX.sym("omega_dot")
        self.shoe = cas.SX.sym("shoe") # Angular velocity of pendulum wheel
        self.shoe_dot = cas.SX.sym("shoe_dot") # Angular acceleration of pendulum wheel
        self.theta_dot = self.omega
        self.tau_m = cas.SX.sym("tau_m") # torque of the reaction wheel
        self.t = cas.SX.sym("t") # time

        # Define ODE
        self.x_1 = cas.vertcat(self.theta,self.omega,self.shoe) # state
        self.ode_1 = cas.vertcat(
            self.omega,
            -(self.B_1 / self.J_1) * self.omega + (self.B_2 / self.J_1) * self.shoe - (1 / self.J_1) * self.tau_m,
            -(self.B_2 / self.J_eq) * self.shoe + (1 / self.J_eq) * self.tau_m
            ) # system of ODEs
        # -------------------------------------------------------------------------------------------

        # -------------------------------------------------------------------------------------------
        # Defining EoM for rolling
        '''
        From Control System for a Spherical Robot
        '''
        # Variables
        self.theta_1 = cas.SX.sym("theta_1") # angle between body reference frame y-axis to inertial reference frame y-axis
        self.theta_2 = cas.SX.sym("theta_2") # angle between theta_1 and pendulum rod
        self.a_1 = cas.SX.sym("a_1") # angular acceleration of angle 1
        self.a_2 = cas.SX.sym("a_2") # angular acceleration of angle 2
        self.v_1 = cas.SX.sym("v_1") # angular velocity of angle 1
        self.v_2 = cas.SX.sym("v_2") # angular velocity of angle 2
        self.Trq = cas.SX.sym("Trq") # Torque of rolling motor
        self.alpha = cas.SX.sym("alpha") # Robot angle with floor

        # Trq = a1 * (J2 - (M2 * r * np.cos(theta_1 + theta_2)) + (M2 * e**2)) + a2 * (J2 + (M2 * e**2)) + (M2 * 9.81 * e * np.sin(theta_1 + theta_2 + alpha))

        # Define ODE
        self.x_2 = cas.vertcat(self.theta_1, self.theta_2, self.v_1, self.v_2) # state
        self.ode_2 = cas.vertcat(
            self.v_1,
            self.v_2,
            (self.Trq - self.a_2 * (self.J_2 + (self.M_2 * self.e**2)) + (self.M_2 * 9.81 * self.e * np.sin(self.theta_1 + self.theta_2 + self.alpha))) / (self.J_2 - (self.M_2 * self.r * np.cos(self.theta_1 + self.theta_2)) + (self.M_2 * self.e**2)),
            self.a_2
            ) # system of ODEs

        self.opti = cas.Opti() # Optimization problem

        self.X_1 = self.opti.variable(3,N) # state trajectory
        self.TAU = self.opti.variable(N) # control trajectory - Rotational Torque
        self.X_2 = self.opti.variable(4,N) # state trajectory
        self.TRQ = self.opti.variable(N) # control trajectory - Linear Torque
        self.A_2 = self.opti.variable(N) # control trajectory - Acceleration of pendulum

        self.T = self.opti.variable() # final time

        self.opti.minimize(self.T) # reach the goal in the shortest time
        self.opti.subject_to(self.X_1[:,0]==[0,0,0]) # Initial condition
        self.opti.subject_to(self.X_1[1,:]<=5) # Enforce velocity limit
        self.opti.subject_to(self.TAU<=1) # Enforce control limit
        self.opti.subject_to(self.TAU>=-1) # Enforce control limit
        self.opti.subject_to(self.X_2[:,0]==[0,0,0,0]) # Initial condition
        self.opti.subject_to(self.X_2[2,self.N-1]==0) # Final velocity 1
        self.opti.subject_to(self.X_2[3,self.N-1]==0) # Final velocity 2
        self.opti.subject_to(self.T>=0) # Time must be positive
            
        self.ode_function_1 = cas.Function('ode_function', [self.x_1, self.tau_m], [self.ode_1])
        self.ode_function_2 = cas.Function('ode_function', [self.x_2, self.Trq, self.alpha, self.a_2], [self.ode_2])

        dt = self.T/self.N
        for i in range(N-1):
            # Foward Euler Method
            x_1_dot_i = self.ode_function_1(self.X_1[:,i], self.TAU[i])
            x_1_next = self.X_1[:,i] + dt * x_1_dot_i
            self.opti.subject_to(self.X_1[:,i+1]==x_1_next) #close the integration gaps for x_1
            x_2_dot_i = self.ode_function_2(self.X_2[:,i], self.TRQ[i], self.alpha_list[i], self.A_2[i])
            x_2_next = self.X_2[:,i] + dt * x_2_dot_i
            self.opti.subject_to(self.X_2[:,i+1]==x_2_next) #close the integration gaps for x_1
        
        self.linear_x = [0] * self.N
        self.linear_y = [0] * self.N
        self.linear_z = [0] * self.N
        self.angular_x = [0] * self.N
        self.angular_y = [0] * self.N
        self.angular_z = [0] * self.N