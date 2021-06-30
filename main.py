import numpy as np
import tinyik


class TwoJointRobot:
    def _init_(self):
        self.number_of_joints = 2
        self.joint_velocity_limts = np.array([2, 2])
        self.joint_accleration_limts = np.array([0.3, 0.3])
        self.l1 = 1
        self.l2 = 1
        self.arm = tinyik.Actuator(['z', [self.l1, 0, 0], 'z', [self.l2, 0, 0]])

    def forward_kinematics(self, joint_positions):
        self.arm.angles = joint_positions
        return self.arm.ee[0:2]

    def inverse_kinematics(self, xy_positions):
        # Z coord required
        self.arm.ee = xy_positions.append(0)
        return self.arm.angles


