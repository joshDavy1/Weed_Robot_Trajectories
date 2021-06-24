import numpy as np
import matplotlib.pyplot as plt
from numpy.core.shape_base import block

velocity_y = -0.5

class WeedRobot:
    def __init__(self):
        pass

    def get_parabola_parameters(self,start_state,start_time, goal_state, goal_time):
        t0 = start_time
        tf = start_time + goal_time
        time_array = np.array([ [1, t0, t0**2, t0**3, t0**4, t0**5],
                                [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
                                [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
                                [1, tf, tf**2, tf**3, tf**4, tf**5],
                                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
                                ])
        time_array_inv = np.linalg.pinv(time_array)

        x_input_array = np.array([[start_state[0][0]],
                                 [start_state[1][0]],
                                 [start_state[2][0]],
                                 [goal_state[0][0]],
                                 [goal_state[1][0]],
                                 [goal_state[2][0]]
        ])
        
        y_input_array = np.array([[start_state[0][1]],
                                 [start_state[1][1]],
                                 [start_state[2][1]],
                                 [goal_state[0][1]],
                                 [goal_state[1][1]],
                                 [goal_state[2][1]]
        ])
        x_parameters = np.matmul(time_array_inv, x_input_array)
        y_parameters = np.matmul(time_array_inv, y_input_array)
        return x_parameters, y_parameters

    def parabola(self, parameters, t):
        terms = [1, t, t**2, t**3, t**4, t**5]
        return np.dot(terms, parameters)

    def get_line_parameters(self,start_state, start_time, goal_pose, goal_time):
        t0 = start_time
        tf = start_time + goal_time
        x0 = start_state[0][0]
        y0 = start_state[0][1]
        xf = goal_pose[0]
        yf = goal_pose[1]
        vx = (xf-x0)/(tf-t0)
        vy = (yf-y0)/(tf-t0)
        x_parameters = np.array([vx, x0])
        y_parameters = np.array([vy, y0])
        return x_parameters, y_parameters

    def plot_parabola_trajectory(self, x_parameters, y_parameters, start_time, goal_time):
        t = np.arange(start_time, start_time + goal_time, 0.1)
        x_values = self.parabola(x_parameters, t)
        y_values = self.parabola(y_parameters, t)
        plt.plot(x_values[0], y_values[0])

    def get_line_end_state(self,start_state, start_time, goal_pose, goal_time):
        t0 = start_time
        tf = start_time + goal_time
        x0 = start_state[0][0]
        y0 = start_state[0][1]
        xf = goal_pose[0]
        yf = goal_pose[1]
        vx = (xf-x0)/(tf-t0)
        vy = (yf-y0)/(tf-t0)
        return np.array([[xf, yf], [vx, vy], [0, 0]])

    def line_trajectory(self, parameters, t):
        # x = vt + x_0
        return parameters[0]*t + parameters[1]

    def plot_line_trajectory(self, x_parameters, y_parameters, start_time, goal_time):
        t = np.arange(start_time, start_time + goal_time, 0.1)
        x_values = self.line_trajectory(x_parameters, t - start_time)
        y_values = self.line_trajectory(y_parameters, t - start_time)
        plt.plot(x_values, y_values)


    def combined_trajectory(self, start_state, weed_pose, start_time, travel_time, spray_time, plot = True):
        """ Returns End State """
        weed_state = np.array([weed_pose, [0, velocity_y], [0, 0]])
        x_parabola_p, y_parabola_p = self.get_parabola_parameters(start_state, start_time, weed_state, travel_time)
        if plot:
            self.plot_parabola_trajectory(x_parabola_p , y_parabola_p, start_time, travel_time)
        # End state of parabola == weed State
        
        goal_pose = weed_pose + np.array([0, spray_time*velocity_y])
        plt.plot(goal_pose[0], goal_pose[1])

        x_line_p, y_line_p = self.get_line_parameters(weed_state, start_time + travel_time, goal_pose, spray_time)
        end_state = self.get_line_end_state(weed_state, start_time + travel_time, goal_pose, spray_time)
        plt.plot(end_state[0][0], end_state[0][1], 'x')

        if plot:
            self.plot_line_trajectory(x_line_p , y_line_p, start_time + travel_time, spray_time)

        def trajectory(t):
            if t < start_time or t > start_time + travel_time + spray_time:
                return None
            elif t >= start_time and t < start_time + travel_time:
                # Parabolic phase
                x = self.parabola(x_parabola_p, t)
                y = self.parabola(y_parabola_p, t)
                return x, y
            elif t >= start_time + travel_time and t < start_time + travel_time + spray_time:
                # Linear phase
                x = self.line_trajectory(x_line_p, t - start_time - travel_time)
                y = self.line_trajectory(y_line_p, t - start_time - travel_time)
                return x, y

        return end_state, trajectory

    def generate_full_trajectory(self, start_state, weeds, start_time, travel_time, spray_time, plot = True):
        time = start_time
        state = start_state
        trajectory_functions = []
        for weed in weeds:
            state, traj = self.combined_trajectory(state, weed, time, travel_time, spray_time, plot)
            time = time + travel_time + spray_time
            trajectory_functions.append([traj,time])

        
        def full_trajectory(t):
            for traj,time in trajectory_functions:
                if t < time:
                    section = traj
                    return traj(t)
            return None
                
        return full_trajectory

    def main(self):
        plt.clf()
        plt.axis('scaled')
        plt.title("Weed Robot")
        plt.axis([-5, 5, -2, 5])

        start = [0, 0]
        state = np.array([start, [0, 0], [0, 0]])
        weeds = np.array([[2, 3], [-1.5, 2.5],[1.5, 0]])
        plt.plot(weeds[:,0],weeds[:,1],'x')
        
        # Move and Spray Weed 1
        time = 0
        travel_time = 5
        spray_time = 6
        traj = self.generate_full_trajectory(state, weeds, time, travel_time, spray_time)

        plt.show(block=False)
        
        t = np.arange(time, len(weeds)*(time + travel_time + spray_time), 0.1)
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        for i in range(len(t)):
            trajectory = traj(t[i])
            x[i] = trajectory[0]
            y[i] = trajectory[1]

        plt.figure()
        plt.plot(x,y)
        plt.figure()
        plt.plot(t[0:-2],np.diff(np.diff(y)))
        plt.show()



w = WeedRobot()
w.main()



        


    