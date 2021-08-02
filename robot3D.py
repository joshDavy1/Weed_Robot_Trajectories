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
                    rtb.RevoluteDH(a=self.link_lengths[0]),
                    rtb.RevoluteDH(a=self.link_lengths[1]),
                    rtb.RevoluteDH(a=self.link_lengths[2]),
                ], name="ibex"
                        )

class ThreeJointRobot:
    def __init__(self):
        self.ibex = Ibex()
        self.number_of_joints = 3
        self.joint_velocity_limits = np.array([3, 3, 3])
        self.joint_accleration_limits = np.array([3, 3, 3])
        self.joint_angles = self.inverse_kinematics((1, 1, 1))
        self.weeds = None

    def forward_kinematics(self, joint_positions):
        return self.forward_kinematics_full(joint_positions)[:-1]

    def inverse_kinematics(self, xy_positions):
        xyz = np.concatenate((xy_positions, [0]))
        return self.inverse_kinematics_full(xyz)[:-1]

    def forward_kinematics_full(self, joint_positions):
        l = self.ibex.link_lengths
        theta = joint_positions[0]
        z = -(l[0]*np.sin(joint_positions[1]) + l[1]*np.sin(joint_positions[1] + joint_positions[2]) + l[2])
        r = l[0]*np.cos(joint_positions[1]) + l[1]*np.cos(joint_positions[1] + joint_positions[2])
        x = r*np.cos(theta)
        y = r*np.sin(theta)
        return np.array([x, y, z])

    def inverse_kinematics_full(self, xyz_positions):
        l = self.ibex.link_lengths
        x = xyz_positions[0]
        y = xyz_positions[1]
        z = xyz_positions[2]
        theta1 = np.arctan2(y, x)
        r = np.sqrt(x**2+y**2)
        cos_theta_2 = (r**2 + (-z-l[2])**2 - l[0]**2 - l[1]**2) / (2*l[0]*l[1])
        theta3 = np.arccos(cos_theta_2)
        theta2 = np.arctan2(-z-l[2], r) - np.arctan2(l[1]*np.sin(theta3), (l[0] + l[1]*np.cos(theta3)) ) 
        theta4 = -theta2 -theta3 + np.pi/2
        return np.array([theta1, theta2, theta3, theta4])

    def check_limits_position(self, xyz_positions):
        x = xyz_positions[0]
        y = xyz_positions[1]
        if x**2 + y**2 > 3.5**2:
            return False
        else:
            return True

    def draw_robot(self, trajectory, weeds, velocity):
        n = trajectory.shape[1]
        trajectory = np.concatenate((trajectory, np.zeros((1, n))))
        for i in range(n):
            trajectory[3, i] = -trajectory[1, i] - trajectory[2, i] + np.pi/2
        env = self.ibex._get_graphical_backend("pyplot")
        env.launch(self.ibex.name + " Trajectory Plot", limits=(-4, 4, -4, 4, 0, 2), fig=None)
        env.add(self.ibex)
        for i in range(n):
            v = np.array([[0],
                    [velocity]])
            if not weeds.size == 0:
                weeds = weeds + v*0.1

            self.plot_weeds(weeds)
            self.ibex.q = trajectory[:, i]
            env.step(0.1)
            
    def plot_weeds(self, weeds):
        if self.weeds:
            x = self.weeds.pop(0)
            x.remove()
        self.weeds = plt.plot(weeds[0, :], weeds[1, :], 'bx')
        

if __name__ == "__main__":
    t = ThreeJointRobot()
    x = t.inverse_kinematics((1.5,-0.7, 2))
    print(x)
    print(t.forward_kinematics(x))
    input()