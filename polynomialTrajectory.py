import numpy as np

class PolynomialTrajectory:
    def __init__(self, robot_def, start_state_xy, goal_state_xy, t0, tf, dt=0.1):
        self.forward_kinematics_position = robot_def.forward_kinematics
        self.inverse_kinematics_position = robot_def.inverse_kinematics
        self.joint_velocity_limits = robot_def.joint_velocity_limits
        self.joint_accleration_limits = robot_def.joint_accleration_limits
        self.number_of_joints = robot_def.number_of_joints

        self.t0 = t0
        self.tf = tf
        self.dt = dt

        self.start_state_xy = start_state_xy
        self.goal_state_xy = goal_state_xy
        self.start_state = self.__convert_state_to_joint_space(start_state_xy)
        self.goal_state = self.__convert_state_to_joint_space(goal_state_xy)
        ## ADD ANGLE CHECK 
        ## Compare clockwise and anticlockwise distance
        ## Whichever is shorter
        ## Make trajectory use that direction
        
        self.all_parameters = []
        for i in range(self.number_of_joints):
            start = self.__get_single_joint_state(self.start_state, i)
            goal = self.__get_single_joint_state(self.goal_state, i)
            self.all_parameters.append(self.__get_polynomial_parameters(t0, tf, start, goal))

    def __get_polynomial_parameters(self, t0, tf, start, goal):
            # q(t) = a + bt + ct^2 + dt^3 + et^4 + ft^5
            # Fifth degree polynomial as we have 6 constraints to fit to:
            # inital position, initial velocity, initial accleration
            # final position, final velocity, final accleration
            # Forms a set of 6 linear equations which we solve
            # Gives us the parameter vector
            time_array = np.array([ [1, t0, t0**2, t0**3, t0**4, t0**5],
                                    [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                                    [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                                    [1, tf, tf**2, tf**3, tf**4, tf**5],
                                    [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                                    [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
                                    ])
            x_input_array = np.array([[start[0]],
                                    [start[1]],
                                    [start[2]],
                                    [goal[0]],
                                    [goal[1]],
                                    [goal[2]]
            ])
            time_array_inv = np.linalg.pinv(time_array)
            return np.matmul(time_array_inv, x_input_array)
             
    def __polynomial(self, parameters, t):
        # Evaluate 5th degree polynomial at t
        terms = np.array([1, t, t**2, t**3, t**4, t**5])
        return np.dot(terms, parameters)
    
    def __inverse_kinematics_velocity(self, velocity_xy, position_xy, dt = 0.005):
        # Estimate joint velocities from IK
        position = self.inverse_kinematics_position(position_xy)
                                                            # x = x_0  + vt
        position_after_dt = self.inverse_kinematics_position(position_xy + velocity_xy*dt)
        # v = (x(t+dt) - x(t))/dt
        return (position_after_dt-position)/dt
    
    def __inverse_kinematics_accleration(self, accleration_xy, velocity_xy, position_xy, dt = 0.005):
        # Estimate joint acclerations from IK
        velocity = self.__inverse_kinematics_velocity(velocity_xy, position_xy)
        velocity_after_dt = self.__inverse_kinematics_velocity(velocity_xy + accleration_xy*dt,
                                                            # v = v_0 + at
                                                             position_xy + velocity_xy*dt+0.5*accleration_xy*dt**2)
                                                             #x = x_o + vt + 1/2at^2
        # a = (v(t+dt) - v(t))/dt                                                     
        return (velocity_after_dt-velocity)/dt

    def __convert_state_to_joint_space(self, state_xy):
        # Convert state in cartesian space to joint space
        state = {}
        state['position'] = self.inverse_kinematics_position(state_xy['position'])
        state['velocity'] = self.__inverse_kinematics_velocity(state_xy['velocity'],
                                                             state_xy['position'])
        state['accleration'] = self.__inverse_kinematics_accleration(state_xy['accleration'],
                                                                   state_xy['velocity'],
                                                                   state_xy['position'])
        return state

    def __get_single_joint_state(self, state, joint):
        # Retrieves a singlular joint state from the state.
        single_state = np.zeros((3))
        single_state[0] = state['position'][joint]
        single_state[1] = state['velocity'][joint]
        single_state[2] = state['accleration'][joint]
        return single_state
    
    def __check_in_velocity_limit(self, parameters, t0, tf, velocity_limit):
        # Function uses parameters defined the other direction
        parameters_flipped = np.flip(parameters.T)[0]
        # Finds stationary points for velocity
        # d^2y/d^2t  = 0
        poly = np.polyder(parameters_flipped, 2)
        roots = np.round(np.roots(poly),2)
        # for each stationary point
        for t in roots:
            # within the time frame
            if t0 <= t <= tf:
                # Calculate instantaneous velocity
                velocity_poly = np.polyder(parameters_flipped, 1)
                velocity = np.polyval(velocity_poly, t)
                # if above limit
                if np.abs(velocity) >= velocity_limit:
                    return False
        return True

    def __check_in_accleration_limit(self, parameters, t0, tf, accleration_limit, dt = 0.005):
        # Function uses parameters defined the other direction
        parameters_flipped = np.flip(parameters.T)[0]
        # Finds stationary points for accleration
        # d^3y/d^3t  = 0
        poly = np.polyder(parameters_flipped, 3)
        roots = np.round(np.roots(poly),4)
        # for each stationary point
        for t in roots:
            # within the time frame
            if t0 <= t <= tf:
                # Calculate instantenous accleration
                accleration_poly = np.polyder(parameters_flipped, 2)
                accleration = np.polyval(accleration_poly, t)
                # if above limit
                if np.abs(accleration) >= accleration_limit:
                    return False
        return True

    ## Public Functions
    def check_limits(self):
        # Check within the limits (Computationally cheap compared to full trajectory
        # generation)
        in_velocity_limits = True
        in_accleration_limits = True
        for joint in range(self.number_of_joints):
            if not self.__check_in_velocity_limit(self.all_parameters[joint],
                                                self.t0, 
                                                self.tf, 
                                                self.joint_velocity_limits[joint]):
                in_velocity_limits = False

            if not self.__check_in_accleration_limit(self.all_parameters[joint],
                                                   self.t0, 
                                                   self.tf, 
                                                   self.joint_accleration_limits[joint]):
                in_accleration_limits = False
        return in_velocity_limits, in_accleration_limits
    
    def sample_trajectory(self, t):
        # Sample trajectory at t
        if self.t0 <= t <= self.tf:
            trajectory = np.zeros(self.number_of_joints)
            for joint in range(self.number_of_joints):
                trajectory[joint] = self.__polynomial(self.all_parameters[joint], t)
            return trajectory
        else:
            return None

    def generate_trajectory(self):
        # Generates the full trajectory with interval dt
        time = np.arange(self.t0, self.tf, self.dt)
        intervals = time.shape[0]
        trajectory = np.zeros((self.number_of_joints, intervals))
        for joint in range(self.number_of_joints):
            for i in range(intervals):
                trajectory[joint, i] = self.__polynomial(self.all_parameters[joint], time[i])
        return trajectory, time

