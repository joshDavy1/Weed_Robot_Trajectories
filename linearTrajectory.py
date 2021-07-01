import numpy as np

class LinearTrajectory:
    def __init__(self, robot_def, start_state_xy, t0, tf):
        self.forward_kinematics_position = robot_def.forward_kinematics
        self.inverse_kinematics_position = robot_def.inverse_kinematics
        self.joint_velocity_limits = robot_def.joint_velocity_limits
        self.joint_accleration_limits = robot_def.joint_accleration_limits
        self.number_of_joints = robot_def.number_of_joints

        self.t0 = t0
        self.tf = tf

        self.start_state_xy = start_state_xy
        self.velocity = self.start_state_xy['velocity']

        self.goal_state_xy = self.start_state_xy
        self.goal_state_xy['position'] += [self.velocity[0]*(tf-t0),
                                           self.velocity[1]*(tf-t0)]

        self.all_parameters = []
        for i in range(2):
            start = self.start_state_xy['position'][i]
            self.all_parameters.append(self.get_linear_parameters(start, self.velocity[i]))

        
    def get_linear_parameters(self, start, velocity):
        # constant velocity model, two parameters
        # x = x_0 + vt
        return np.array([start, velocity])

    def linear_model(self, parameters, t):
        # x = x_0 + vt
        return parameters[0] + parameters[1]*t

    def inverse_kinematics_velocity(self, velocity_xy, position_xy, dt = 0.005):
        position = self.inverse_kinematics_position(position_xy)
                                                            # x = x_0  + vt
        position_after_dt = self.inverse_kinematics_position(position_xy + velocity_xy*dt)
        # v = (x(t+dt) - x(t))/dt
        return (position_after_dt-position)/dt
    
    def inverse_kinematics_accleration(self, accleration_xy, velocity_xy, position_xy, dt = 0.005):
        velocity = self.inverse_kinematics_velocity(velocity_xy, position_xy)
        velocity_after_dt = self.inverse_kinematics_velocity(velocity_xy + accleration_xy*dt,
                                                            # v = v_0 + at
                                                             position_xy + velocity_xy*dt+0.5*accleration_xy*dt**2)
                                                             #x = x_o + vt + 1/2at^2
        # a = (v(t+dt) - v(t))/dt                                                     
        return (velocity_after_dt-velocity)/dt
    
    def check_in_velocity_limit(self, parameters, t0, tf, velocity_limit, dt = 0.1):
        time = np.arange(t0, tf, dt)
        for t in time:
            cartesian_position = self.linear_model(parameters, t)
            cartesian_velocity = self.velocity
            joint_space_velocity = self.inverse_kinematics_velocity(cartesian_velocity, cartesian_position)
            if joint_space_velocity >= velocity_limit:
                return False
        return True

    def check_in_accleration_limit(self, parameters, t0, tf, accleration_limit, dt = 0.1):
        time = np.arange(t0, tf, dt)
        for t in time:
            cartesian_position = self.linear_model(parameters, t)
            cartesian_velocity = self.velocity
            cartesian_accleration = 0
            joint_space_accleration = self.inverse_kinematics_accleration(cartesian_accleration,
                                                                          cartesian_velocity,
                                                                          cartesian_position)
            if joint_space_accleration >= accleration_limit:
                return False
        return True
        
    ## Public Functions

    def check_limits(self):
        in_velocity_limits = True
        in_accleration_limits = True

        for joint in range(self.number_of_joints):
            if not self.check_in_velocity_limit(self.all_parameters[joint],
                                                self.t0, 
                                                self.tf, 
                                                self.joint_velocity_limits[joint]):
                in_velocity_limits = False

            if not self.check_in_accleration_limit(self.all_parameters[joint],
                                                   self.t0, 
                                                   self.tf, 
                                                   self.joint_accleration_limits[joint]):
                in_accleration_limits = False
        return in_velocity_limits, in_accleration_limits
    
    def sample_trajectory(self, t):
        trajectory = np.zeros((self.number_of_joints, 1))
        if self.t0 < t < self.tf:
            for joint in range(self.number_of_joints):
                trajectory[joint] = self.inverse_kinematics_position(self.linear_model(self.all_parameters[joint], t))
            return trajectory
        else:
            return None

    def generate_trajectory(self, dt = 0.1):
        time = np.arange(self.t0, self.tf, dt)
        intervals = len(time)
        trajectory = np.zeros((self.number_of_joints, intervals))
        for joint in range(self.number_of_joints):
            for t in range(intervals):
                trajectory[joint, t] = self.inverse_kinematics_position(self.linear_model(self.all_parameters[joint], t))


    def get_goal_state_xy(self):
        return self.goal_state_xy