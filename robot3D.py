import tinyik
import numpy as np
import matplotlib.pyplot as plt

class ThreeJointRobot:
    def __init__(self):
        self.number_of_joints = 3
        self.joint_velocity_limits = np.array([1, 1, 1])
        self.joint_accleration_limits = np.array([1, 1, 1])
        self.link_lengths = np.array([3, 2, 1])
        self.arm = tinyik.Actuator([[1, .0, .0], 'y', [1, .0, .0], 'x', [.0, 1, .0], 'z', [.0, 1, .0]])
        # tinyik.visualize(self.arm)
        # input()
        self.joint_angles = self.inverse_kinematics(np.array([1, 1]))

        fig = plt.figure(figsize=(6, 12), dpi=80)
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
        if xy_positions[0]**2 +xy_positions[1]**2 <= 5:
            return True
        return False

    def draw(self, weeds):
        # Variables
        # Set up plot
        plt.clf()
        plt.axis('scaled')
        plt.title("Robot")
        plt.axis([-5, 5, -2, 15])
        end_effector = self.forward_kinematics(self.joint_angles)
        plt.plot(end_effector[0], end_effector[1], 'o')
        #tinyik.visualize(self.arm)
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
    
    