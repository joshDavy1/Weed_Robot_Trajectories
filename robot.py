import numpy as np
import matplotlib.pyplot as plt
from numpy.core.shape_base import block
import tinyik
from trajectoryGenerator import TrajectoryGenerator

class Robot:
    def __init__(self):
        self.l1 = 3
        self.l2 = 3
        
        self.v1_limit = 0.5
        self.a1_limit = 0.3
        self.v2_limit = 0.5
        self.a2_limit = 0.3

        self.arm = tinyik.Actuator(['z', [self.l1, 0, 0], 'z', [self.l2, 0, 0]])

    def forward_kinematics(self, theta1, theta2):
        self.arm.angles = [theta1, theta2]
        return self.arm.ee[0:2]

    def inverse_kinematics(self, x, y):
        self.arm.ee = [x, y, 0]
        return self.arm.angles

    def draw(self, theta1, theta2, weeds):
        plt.clf()
        plt.axis('scaled')
        plt.title("Robot")
        plt.axis([-5, 5, -2, 7])
        # Draw Arm
        x1 = self.l1*np.cos(theta1)
        y1 = self.l1*np.sin(theta1)
        x2 = self.l2*np.cos(theta1 + theta2) + x1
        y2 = self.l2*np.sin(theta1 + theta2) + y1
        line = plt.Line2D((0, x1), (0, y1), lw=1, c='black', marker='.', 
                         markersize=2, 
                         markerfacecolor='r', 
                         markeredgecolor='r')
        plt.gca().add_line(line)
        line = plt.Line2D((x1, x2), (y1, y2), lw=1, c='black', marker='.', 
                         markersize=10, 
                         markerfacecolor='r', 
                         markeredgecolor='r')
        plt.gca().add_line(line)

        # Draw Weeds
        plt.plot(weeds[:,0], weeds[:,1], 'x')

        plt.show(block=False)
        plt.pause(0.1)

    def find_maximums_joint_space(self, traj, start_time, end_time, interval):
        t = np.arange(start_time, end_time, interval)
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        theta1s = np.zeros_like(t)
        theta2s = np.zeros_like(t)
        plt.figure()
        for i in range(len(t)):
            trajectory = traj(t[i])
            x[i] = trajectory[0]
            y[i] = trajectory[1]
            theta1, theta2 = self.inverse_kinematics(trajectory[0], trajectory[1])
            theta1s[i] = theta1
            theta2s[i] = theta2
        dtheta1s = np.diff(theta1s)/interval
        dtheta2s = np.diff(theta2s)/interval
        ddtheta1s = np.diff(dtheta1s)/interval
        ddtheta2s = np.diff(dtheta2s)/interval
        plt.plot(ddtheta2s)
        max_v1 = np.max(np.abs(dtheta1s))
        max_v2 = np.max(np.abs(dtheta2s))
        max_a1 = np.max(np.abs(ddtheta1s))
        max_a2 = np.max(np.abs(ddtheta2s))
        return np.array([[max_v1, max_v2], [max_a1, max_a2]])

    def generate_feasible_trajectory(self, start_state, weed_state, start_time, spray_time, velocity):
        """ Returns a trajectory to spray a single weed given joint limits.
            If not possible returns None """
        # Find first possible intercept point i.e y at max
        # Find time it will happen at
        # Generate a trajectory to see if possible
        # If possible run
        # If not then why
            # if limited by travel_time (i.e. breaks constraints in travel to intercept point
                # increase travel_time (changes intercept point)
            # if limited by spray_time ((i.e. breaks constraints in tracking of weed)
                # unfeasible to spray (bit of a assumption but good enough)
        

    def main(self):
        plt.close('all')
        weeds = np.array([[2, 3], [-1.5, 2.5],[1, 3], [-2, 4]])
        velocity = -0.5

        traGenerator =  TrajectoryGenerator()

        theta1 = 0
        theta2 = np.pi*2/3
        x,y = self.forward_kinematics(theta1, theta2)
        start_state = np.array([[x, y], [0, 0], [0, 0]])
        start_time = 0
        travel_time = 5
        spray_time = 3
        traj = traGenerator.generate_full_trajectory(start_state, weeds, start_time, travel_time, spray_time, velocity, plot = True)
        print(self.find_maximums_joint_space(traj, start_time, len(weeds)*(start_time + travel_time + spray_time), 0.3))

        t = np.arange(start_time, len(weeds)*(start_time + travel_time + spray_time), 0.2)
        x = np.zeros_like(t)
        y = np.zeros_like(t)
        theta1s = np.zeros_like(t)
        theta2s = np.zeros_like(t)
        plt.figure()
        for i in range(len(t)):
            trajectory = traj(t[i])
            x[i] = trajectory[0]
            y[i] = trajectory[1]
            theta1, theta2 = self.inverse_kinematics(trajectory[0], trajectory[1])
            theta1s[i] = theta1
            theta2s[i] = theta2
            self.draw(theta1, theta2, weeds)
        plt.figure()
        plt.plot(theta1s, theta2s)
        plt.show(block=False)
        input()

        

r = Robot()
r.main()
