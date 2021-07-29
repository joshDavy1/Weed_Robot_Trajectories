import numpy as np
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import *

class Ibex(rtb.DHRobot):
    def __init__(self):
        self.link_lengths = np.array([2, 2, 0.5])
        super().__init__(
                [
                    rtb.RevoluteDH(d=0, alpha=-np.pi/2),
                    rtb.RevoluteDH(a=self.link_lengths[0], alpha=0),
                    rtb.RevoluteDH(a=self.link_lengths[1], alpha=0),
                    rtb.RevoluteDH(a=self.link_lengths[2]),
                ], name="ibex"
                        )

class ThreeJointRobot:
    def __init__(self):
        self.number_of_joints = 3
        self.joint_velocity_limits = np.array([1, 1, 1])
        self.joint_accleration_limits = np.array([1, 1, 1])
        self.ibex = Ibex()
        print(self.ibex)
        # self.weed_fig = plt.figure(figsize=(12, 12), dpi=80)
        # self.weed_fig.canvas.mpl_connect('button_press_event', self.onclick)
        # self.robot_fig = plt.fig
        # self.read_click = True
        # self.last_clicked = np.empty((2,1))

    def forward_kinematics(self, joint_positions):
        l = self.ibex.link_lengths
        theta = joint_positions[0]
        z = l[0]*np.sin(joint_positions[1]) + l[1]*np.sin(joint_positions[1]+joint_positions[2]) + l[2]
        r = l[0]*np.cos(joint_positions[1]) + l[1]*np.cos(joint_positions[1]+joint_positions[2])
        x = r*np.cos(theta)
        y = r*np.sin(theta)
        return np.array([x, y, z])

    def inverse_kinematics(self, xyz_positions):
        l = self.ibex.link_lengths

        x = xyz_positions[0]
        y = xyz_positions[1]
        z = xyz_positions[2]
        theta1 = np.arctan2(y, x)
        r = x/np.cos(theta1)

        cos_theta_2 = (r**2 + (z-l[2])**2 - l[0]**2 - l[1]**2) / (2*l[0]*l[1])
        theta3 = np.arccos(cos_theta_2)
        theta2 = np.arctan2(z-l[2], r)  - np.arctan2(l[1]*np.sin(theta3), \
             (l[0] + l[1]*np.cos(theta3)) ) 
        theta4 = -theta2 -theta3 +np.pi/2

        return np.array([theta1, theta2, theta3, theta4])


t = ThreeJointRobot()
x =(0, 0, 0, np.pi/2)

q = t.inverse_kinematics((2,1, 0))
print(q)

print(t.forward_kinematics(q))
t.ibex.plot(q)
input()


    # def check_limits_position(self, xy_positions):
    #     pass

    # def draw_robot(self):
    #     pass


    # def draw_weeds(self, weeds):
    #     # Variables
    #     # Set up plot
    #     plt.clf()
    #     plt.axis('scaled')
    #     plt.title("Robot")
    #     plt.axis([-5, 5, -2, 15])
    #     end_effector = self.forward_kinematics(self.joint_angles)
    #     plt.plot(end_effector[0], end_effector[1], 'o')
    #     # Draw Weeds
    #     if not weeds.size == 0:
    #         plt.plot(weeds[0,:], weeds[1,:], 'x')
    #     # Show and delay
    #     plt.show(block=False)
    #     plt.pause(0.1)
    
    # def onclick(self, event):
    #     ix, iy = event.xdata, event.ydata
    #     self.last_clicked = np.array([[ix], [iy]])
    #     self.read_click = False
    
    