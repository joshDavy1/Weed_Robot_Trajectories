import numpy as np
import matplotlib.pyplot as plt
import tinyik
from polynomialTrajectory import PolynomialTrajectory
from linearTrajectory import LinearTrajectory

class TwoJointRobot:
    def __init__(self):
        self.number_of_joints = 2
        self.joint_velocity_limits = np.array([0.5, 0.5])
        self.joint_accleration_limits = np.array([0.5, 0.5])
        self.link_lengths = np.array([3, 3])
        self.arm = tinyik.Actuator(['z', [self.link_lengths[0], 0, 0],
                                    'z', [self.link_lengths[1], 0, 0]])

    def forward_kinematics(self, joint_positions):
        self.arm.angles = joint_positions
        return self.arm.ee[0:2]

    def inverse_kinematics(self, xy_positions):
        if xy_positions[0]**2 +xy_positions[1]**2 >= \
            (self.link_lengths[0]+self.link_lengths[1])**2:
            raise Exception("Error: Beyond Arm Reach")
        # Z coord required
        self.arm.ee = np.concatenate((xy_positions, [0]))
        return self.arm.angles


class Main:
    def __init__(self):
        self.velocity = -0.5

    def onclick(self, event):
        ix, iy = event.xdata, event.ydata
        self.weeds = np.concatenate((self.weeds, np.array([[ix], [iy]])), axis = 1)

    def draw(self, robot_def, joint_angles):
        # Variables
        theta1 = joint_angles[0]
        theta2 = joint_angles[1]
        l1 = robot_def.link_lengths[0]
        l2 = robot_def.link_lengths[1]
        # Set up plot
        plt.clf()
        plt.axis('scaled')
        plt.title("Robot")
        plt.axis([-5, 5, -2, 15])
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
        if not self.weeds.size == 0:
            plt.plot(self.weeds[0,:], self.weeds[1,:], 'x')
        # Show and delay
        plt.show(block=False)
        plt.pause(0.1)

    def update(self):
        # Update weed positions
        v = np.array([[0],
                    [self.velocity]])
        self.weeds = self.weeds + v*0.1


    def main(self):
        plt.close('all')
        robot_def = TwoJointRobot()
    
        start_state_xy = {}
        start_state_xy['position'] = np.array([1, 1])
        start_state_xy['velocity'] = np.array([0, 0])
        start_state_xy['accleration'] = np.array([0, 0])


        weed_state_xy = {}
        weed_state_xy['position'] = np.array([2, 3])
        weed_state_xy['velocity'] = np.array([0, self.velocity])
        weed_state_xy['accleration'] = np.array([0, 0])

        start_time = 0
        travel_time = 5
        spray_time = 3

        parabolic_trajectory = PolynomialTrajectory(robot_def,
                                          start_state_xy,
                                          weed_state_xy,
                                          start_time,
                                          travel_time)

        check_limits = parabolic_trajectory.check_limits()
        print("Parabolic Velocity Check Passed:", check_limits[0])
        print("Parabolic Accleration Check Passed:", check_limits[1])

        if check_limits[0] and check_limits[1]:
            para_trajectory,_ = parabolic_trajectory.generate_trajectory()
            plt.plot(para_trajectory[0,:], para_trajectory[1,:])

        linear_trajectory = LinearTrajectory(robot_def,
                                             weed_state_xy,
                                             travel_time,
                                             spray_time)
        
        check_limits = linear_trajectory.check_limits()
        print("Linear Velocity Check Passed:", check_limits[0])
        print("Linear Accleration Check Passed:", check_limits[1])

        if check_limits[0] and check_limits[1]:
            lin_trajectory,_ = linear_trajectory.generate_trajectory()
            plt.plot(lin_trajectory[0], lin_trajectory[1])

        plt.show(block=False)
        
        fig = plt.figure(figsize=(6, 12), dpi=80)
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        trajectory = np.concatenate((para_trajectory, lin_trajectory), axis=1)
        self.weeds = np.empty((2,1))
        size = trajectory.shape[1]
        for i in range(size):
            self.draw(robot_def, trajectory[:,i])
            self.update()
        input()


if __name__ == "__main__":
    m = Main()
    m.main()