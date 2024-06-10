import numpy as np
import math


class ManiuplatorModel:
    def __init__(self, Tp):
        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = 0.04
        self.m1 = 3.
        self.l2 = 0.4
        self.r2 = 0.04
        self.m2 = 2.4
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2)
        self.m3 = 0
        self.r3 = 0.05
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2

    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        d1 = self.l1 / 2
        d2 = self.l2 / 2

        q1, q2, q1_dot, q2_dot = x

        c1 = math.cos(q1)
        c2 = math.cos(q2)
        s1 = math.sin(q1)
        s2 = math.sin(q2)

        alpha = self.m1*d1**2 + self.I_1 + self.m2*(self.l1**2 + d2**2) + self.I_2
        beta = self.m2*self.l1*d2
        gamma = self.m2*d2**2 + self.I_2


        # alpha = self.m1*d1**2 + self.I_1 + self.m2*(self.l1**2 + d2**2) + self.I_2 + self.m3*(self.l1**2 + self.l2**2) + self.I_3
        # beta = self.m2*self.l1*d2 + self.m3*self.l1*self.l2
        # gamma = self.m2*d2**2 + self.I_2 + self.m3*self.l2**2 + self.I_3

        M = []
        M.append([alpha + 2*beta*c2, gamma + beta*c2])
        M.append([gamma + beta*c2, gamma])
        return M

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        d1 = self.l1 / 2
        d2 = self.l2 / 2

        q1, q2, q1_dot, q2_dot = x

        c1 = math.cos(q1)
        c2 = math.cos(q2)
        s1 = math.sin(q1)
        s2 = math.sin(q2)

        alpha = self.m1*d1**2 + self.I_1 + self.m2*(self.l1**2 + d2**2) + self.I_2
        beta = self.m2*self.l1*d2
        gamma = self.m2*d2**2 + self.I_2

        # alpha = self.m1*d1**2 + self.I_1 + self.m2*(self.l1**2 + d2**2) + self.I_2 + self.m3*(self.l1**2 + self.l2**2) + self.I_3
        # beta = self.m2*self.l1*d2 + self.m3*self.l1*self.l2
        # gamma = self.m2*d2**2 + self.I_2 + self.m3*self.l2**2 + self.I_3

        C = []
        C.append([-beta*s2*q2_dot, -beta*s2*(q1_dot + q2_dot)])
        C.append([beta*s2*q1_dot, 0])

        return C
