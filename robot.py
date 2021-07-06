import tinyik
import numpy as np
import matplotlib.pyplot as plt

class TwoJointRobot:
    def __init__(self):
        self.number_of_joints = 2
        self.joint_velocity_limits = np.array([2, 2])
        self.joint_accleration_limits = np.array([2, 2])
        self.link_lengths = np.array([3, 3])
        self.arm = tinyik.Actuator(['z', [self.link_lengths[0], 0, 0],
                                    'z', [self.link_lengths[1], 0, 0]])
        self.joint_angles = self.inverse_kinematics(np.array([1, 1]))

        fig = plt.figure(figsize=(8, 12), dpi=80)
        fig.canvas.mpl_connect('button_press_event', self.onclick)
        self.read_click = True
        self.last_clicked = np.empty((2,1))

    def forward_kinematics(self, joint_positions):
        self.arm.angles = joint_positions
        return self.arm.ee[0:2]

    def inverse_kinematics(self, xy_positions):
        if xy_positions[0]**2 +xy_positions[1]**2 >= \
            (self.link_lengths[0]+self.link_lengths[1])**2:
            raise Exception("Beyond Arm Reach")
        # Z coord required
        self.arm.ee = np.concatenate((xy_positions, [0]))
        return self.arm.angles

    def check_limits_position(self, xy_positions):
        # Check in arm reach
        if xy_positions[0]**2 +xy_positions[1]**2 <= \
            (self.link_lengths[0]+self.link_lengths[1])**2:
            return True
        return False

    def draw(self, weeds):
        # Variables
        theta1 = self.joint_angles[0]
        theta2 = self.joint_angles[1]
        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]
        # Set up plot
        plt.clf()
        plt.axis('scaled')
        plt.title("Robot")
        plt.axis([-7, 7, -8, 20])
        # Arm Kinematics
        x1 = l1*np.cos(theta1)
        y1 = l1*np.sin(theta1)
        x2 = l2*np.cos(theta1 + theta2) + x1
        y2 = l2*np.sin(theta1 + theta2) + y1
        # Draw Lines
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
        if not weeds.size == 0:
            plt.plot(weeds[0,:], weeds[1,:], 'x')
        # Show and delay
        plt.show(block=False)
        plt.pause(0.1)
    
    def onclick(self, event):
        ix, iy = event.xdata, event.ydata
        self.last_clicked = np.array([[ix], [iy]])
        self.read_click = False
    
    