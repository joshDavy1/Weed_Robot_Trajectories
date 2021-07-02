import numpy as np

class LinearTrajectory:
    def __init__(self, robot_def, start_state_xy, t0, tf, dt=0.1):
        self.forward_kinematics_position = robot_def.forward_kinematics
        self.inverse_kinematics_position = robot_def.inverse_kinematics
        self.joint_velocity_limits = robot_def.joint_velocity_limits
        self.joint_accleration_limits = robot_def.joint_accleration_limits
        self.number_of_joints = robot_def.number_of_joints

        self.t0 = t0
        self.tf = tf
        self.dt = dt

        self.start_state_xy = start_state_xy
        self.velocity = self.start_state_xy['velocity']

        self.goal_state_xy = self.start_state_xy.copy()
        self.goal_state_xy['position'] = self.goal_state_xy['position'] + \
                                        [self.velocity[0]*(tf-t0), self.velocity[1]*(tf-t0)]

        self.all_parameters = []
        for i in range(2):
            start = self.start_state_xy['position'][i]
            self.all_parameters.append(self.get_linear_parameters(start, self.velocity[i]))

        # Generate Trajectory
        self.time = np.arange(self.t0, self.tf, self.dt)
        print(self.time)
        intervals = self.time.shape[0]
        self.trajectory = np.zeros((self.number_of_joints, intervals))
        for i in range(intervals):
            position_xy = [self.linear_model(self.all_parameters[0], self.time[i]),
                           self.linear_model(self.all_parameters[1], self.time[i])]
            print(position_xy)
            self.trajectory[:, i] = self.inverse_kinematics_position(position_xy).T

        
    def get_linear_parameters(self, start, velocity):
        # constant velocity model, two parameters
        # x = x_0 + vt
        return np.array([start, velocity])

    def linear_model(self, parameters, t):
        # x = x_0 + vt
        return parameters[0] + parameters[1]*(t-self.t0)
    
    ## Public Functions

    def check_limits(self):
        velocity = np.diff(self.trajectory)/self.dt
        accleration = np.diff(velocity)/self.dt
        in_velocity_limits = np.all(velocity <= np.atleast_2d(self.joint_velocity_limits).T)
        in_accleration_limits = np.all(accleration <= np.atleast_2d(self.joint_accleration_limits).T)
        return in_velocity_limits, in_accleration_limits
    
    def sample_trajectory(self, t):
        trajectory = np.zeros((self.number_of_joints, 1))
        if self.t0 < t < self.tf:
            for joint in range(self.number_of_joints):
                trajectory[joint] = self.inverse_kinematics_position(self.linear_model(self.all_parameters[joint], t))
            return trajectory
        else:
            return None

    def generate_trajectory(self):
        return self.trajectory, self.time

    def get_goal_state_xy(self):
        return self.goal_state_xy