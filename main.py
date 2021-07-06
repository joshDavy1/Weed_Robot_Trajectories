import numpy as np
import matplotlib.pyplot as plt
from polynomialTrajectory import PolynomialTrajectory
from linearTrajectory import LinearTrajectory
from robot import TwoJointRobot
from robot3D import ThreeJointRobot

class Main:
    def __init__(self):
        self.velocity = -1
        self.spraying_time = 1
        self.weeds = np.array([[1, 1.1], [7, 7.5]])
        self.x = False

    def get_intercept(self, weed):
        """ Gets the intercept point and time
         i.e. when weed will be at y = 3"""
        dist = 3-weed[1]
        if dist < 0:
            intercept_time = dist/self.velocity
            intercept_point = np.array([weed[0], 3])
        else:
            intercept_time = 1.5
            intercept_point = np.array([weed[0], weed[1]+ self.velocity*intercept_time])
        return intercept_point, intercept_time

    def update(self, weed = np.empty([])):
        """ Update weed list based on velocity 
        and if any new weeds are detected"""
        if not weed.size == 1:
            self.weeds = np.concatenate((self.weeds, weed), axis=1)
        # Update weed positions
        v = np.array([[0],
                    [self.velocity]])
        if not self.weeds.size == 0:
            self.weeds = self.weeds + v*0.1

    def main(self):
        """ Main """
        plt.close('all')
        # Robot definition
        robot_def = TwoJointRobot()
        #robot_def = ThreeJointRobot()
        # Initial state of the end effector
        start_state_xy = {}
        start_state_xy['position'] = robot_def.forward_kinematics(robot_def.joint_angles)
        start_state_xy['velocity'] = np.array([0, 0])
        start_state_xy['accleration'] = np.array([0, 0])
        # Initial time is zero
        time = 0
        # Draw
        robot_def.draw(self.weeds)
        # While still detected weeds
        total_trajectory = np.array([])
        while not self.weeds.size == 0:
            # Select first weed
            selected_weed = self.weeds[:, 0]
            # Get intercept point
            weed_pose, intercept_time = self.get_intercept(selected_weed.T)
            # Times for trajectory generation
            start_time = time
            travel_time = start_time + intercept_time
            spray_time = travel_time + self.spraying_time
            print(start_time, travel_time, spray_time)
            # Check weed state and end point within arm range
            weed_in_range = robot_def.check_limits_position(weed_pose)
            end_in_range = robot_def.check_limits_position((weed_pose[0],
                                                            weed_pose[1]+self.velocity*self.spraying_time))
                                                            
            if not weed_in_range or not end_in_range:
                # Remove weed from list
                self.weeds = np.delete(self.weeds, 0, 1)
                continue
            # State of weed at intercept
            weed_state_xy = {}
            weed_state_xy['position'] = weed_pose
            weed_state_xy['velocity'] = np.array([0, self.velocity])
            weed_state_xy['accleration'] = np.array([0, 0])
            # Poly curve to meet weed at its velocity
            poly_trajectory = PolynomialTrajectory(robot_def,
                                                   start_state_xy,
                                                   weed_state_xy,
                                                   start_time,
                                                   travel_time)
            # Final pose may not be exactly the weed state so update
            weed_state_xy['position'] = robot_def.forward_kinematics(poly_trajectory.sample_trajectory(travel_time))
            print(weed_state_xy)
            # Linear cartesian to track weed
            linear_trajectory = LinearTrajectory(robot_def,
                                                 weed_state_xy,
                                                 travel_time,
                                                 spray_time)

            # Check polynomial trajectoy feasible
            poly_check_limits = poly_trajectory.check_limits()
            print("Parabolic Velocity Check Passed:", poly_check_limits[0])
            print("Parabolic Accleration Check Passed:", poly_check_limits[1])
            # Check linear cartesian feasible
            lin_check_limits = linear_trajectory.check_limits()
            print("Linear Position Check Passed:", lin_check_limits[0])
            print("Linear Velocity Check Passed:", lin_check_limits[1])
            print("Linear Accleration Check Passed:", lin_check_limits[2])
            if np.all(poly_check_limits) and np.all(lin_check_limits):
                # Generate full trajectory
                p,_ = poly_trajectory.generate_trajectory()
                l,_ = linear_trajectory.generate_trajectory()
                trajectory = np.concatenate((p, l), axis=1)
                # Append to full trajectory array
                if total_trajectory.size == 0:
                    total_trajectory = trajectory
                else:
                    total_trajectory = np.concatenate((total_trajectory,trajectory),axis=1)
                # Execute
                length = trajectory.shape[1]
                for i in range(length):
                    # Set angles
                    robot_def.joint_angles = trajectory[:, i]
                    # Update visualisation
                    robot_def.draw(self.weeds)
                    time += 0.1
                    # If a new weed has been detected add to the list
                    if robot_def.read_click == False:
                        self.update(robot_def.last_clicked)
                        robot_def.read_click = True
                    else:
                        self.update()
                # Next start state is last position of the trajectory
                start_state_xy = linear_trajectory.get_goal_state_xy()
            # Remove weed from list
            self.weeds = np.delete(self.weeds, 0, 1)
        
        print(total_trajectory)
        # Plot Joint Trajectories
        n = total_trajectory.shape[1]
        plt.figure()
        plt.subplot(1, 3, 1)
        plt.title("Position")
        plt.plot(total_trajectory[0])
        plt.plot(total_trajectory[1])
        plt.subplot(1, 3, 2)
        plt.title("Velocity")
        plt.plot(np.diff(total_trajectory[0])/0.1)
        plt.plot(np.diff(total_trajectory[1])/0.1)
        plt.subplot(1, 3, 3)
        plt.title("Accleration")
        plt.plot(np.diff(total_trajectory[0],2)/(0.1**2))
        plt.plot(np.diff(total_trajectory[1],2)/(0.1**2))
        plt.show()
if __name__ == "__main__":
    m = Main()
    m.main()