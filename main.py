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
        self.velocity = -1

    def onclick(self, event):
        ix, iy = event.xdata, event.ydata
        self.weeds = np.concatenate((self.weeds, np.array([[ix], [iy]])), axis=1)

    def draw(self, robot_def, joint_angles):
        self.angles = joint_angles
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
        if not self.weeds.size == 0:
            self.weeds = self.weeds + v*0.1

    def get_intercept(self, weed):
        dist = weed[1] - 3
        intercept_time = np.abs(dist/self.velocity)
        intercept_point = np.array([weed[0], 3])
        return intercept_point, intercept_time


    def main(self):
        plt.close('all')
        robot_def = TwoJointRobot()
    
        start_state_xy = {}
        start_state_xy['position'] = np.array([1, 1])
        start_state_xy['velocity'] = np.array([0, 0])
        start_state_xy['accleration'] = np.array([0, 0])

        start_time = 0
        time = 0
        spraying_time = 3

        fig = plt.figure(figsize=(6, 12), dpi=80)
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)

        self.weeds = np.array([[1],[7]])
        self.draw(robot_def, robot_def.inverse_kinematics(start_state_xy['position']))

        while not self.weeds.size == 0:
            # Get next weed to intercept
            # Find intercept point
            # Create trajectory
            # If checks pass
            #   Execute
            print(self.weeds)
            n = self.weeds.shape[1]
            selected_weed = self.weeds[:,0]
            if self.weeds[1,0] < 3:
                # Delete
                self.weeds = np.delete(self.weeds,0,1)
            if self.weeds.size == 0:
                continue

            weed_pose, intercept_time = self.get_intercept(selected_weed.T)
            travel_time = start_time + intercept_time
            spray_time = travel_time + spraying_time
            print(start_time, travel_time, spray_time)
            weed_state_xy = {}
            weed_state_xy['position'] = weed_pose
            weed_state_xy['velocity'] = np.array([0, self.velocity])
            weed_state_xy['accleration'] = np.array([0, 0])

            
            
            parabolic_trajectory = PolynomialTrajectory(robot_def,
                                            start_state_xy,
                                            weed_state_xy,
                                            start_time,
                                            travel_time)
            
            linear_trajectory = LinearTrajectory(robot_def,
                                                weed_state_xy,
                                                travel_time,
                                                spray_time)

            para_check_limits = parabolic_trajectory.check_limits()
            print("Parabolic Velocity Check Passed:", para_check_limits[0])
            print("Parabolic Accleration Check Passed:", para_check_limits[1])

            lin_check_limits = linear_trajectory.check_limits()
            print("Linear Velocity Check Passed:", lin_check_limits[0])
            print("Linear Accleration Check Passed:", lin_check_limits[1])

            #if 1==1:#np.all(para_check_limits) and np.all(lin_check_limits):
            para_trajectory,_ = parabolic_trajectory.generate_trajectory()
            lin_trajectory,_ = linear_trajectory.generate_trajectory()
            #print(para_trajectory, lin_trajectory)
            trajectory = np.concatenate((para_trajectory, lin_trajectory), axis=1)
            # Execute
            size = trajectory.shape[1]
            for i in range(size):
                self.draw(robot_def, trajectory[:,i])
                time += 0.1
                self.update()
            
            

            start_state_xy = linear_trajectory.get_goal_state_xy()
            start_time = time
            
            self.weeds = np.delete(self.weeds,0,1)

            #input()


if __name__ == "__main__":
    m = Main()
    m.main()