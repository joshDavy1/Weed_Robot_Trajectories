import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import *
import numpy as np

class Ibex(rtb.DHRobot):
    def __init__(self):
        self.link_lengths = np.array([2, 2, 1])
        super().__init__(
                [
                    rtb.RevoluteDH(d=1, alpha=np.pi/2),
                    rtb.RevoluteDH(a=self.link_lengths[0], alpha=np.pi),
                    rtb.RevoluteDH(a=self.link_lengths[1]),
                    rtb.RevoluteDH(a=self.link_lengths[2]),
                ], name="ibex"
                        )

i = Ibex()
print(i)
x = np.arange(0, np.pi, np.pi/25)
y = np.zeros(25)
t = np.vstack((y, y, x, y)).T
print(t)
i.plot(t)
input()
