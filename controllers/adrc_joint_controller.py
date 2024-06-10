import numpy as np
from observers.eso import ESO
from .controller import Controller
import math


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd
        self.Tp = Tp

        A = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 0]
        ])
        B = np.array([[0], [b], [0]])
        L = np.array([[3*p],
                      [3*p**2],
                      [p**3]])
        W = np.array([[1, 0, 0]])
        self.eso = ESO(A, B, W, L, q0, Tp)

    def set_b(self, b):
        self.b = b
        B = np.array([[0],
                      [b],
                      [0]])
        self.eso.set_B(B)

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot, i):
        q = x[0]
        z_est = self.eso.get_state()
        q_est = z_est[0]
        q_dot_est = z_est[1]
        f_est = z_est[2]
        e =q_d - q
        e_dot = q_d_dot - q_dot_est
        print("e_dot", e_dot)
        print("e", e)
        v = q_d_ddot + self.kd * e_dot + self.kp * e
        u = (v - f_est) / self.b
        self.eso.update(q, u)

        l1 = 0.5
        r1 = 0.04
        m1 = 3.
        l2 = 0.4
        r2 = 0.04
        m2 = 2.4
        I_1 = 1 / 12 * m1 * (3 * r1 ** 2 + l1 ** 2)
        I_2 = 1 / 12 * m2 * (3 * r2 ** 2 + l2 ** 2)
        m3 = 1
        r3 = 0.05
        I_3 = 2. / 5 * m3 * r3 ** 2

        d1 = l1 / 2
        d2 = l2 / 2

        c1 = math.cos(q_est)
        c2 = math.cos(q_est)
        s1 = math.sin(q_est)
        s2 = math.sin(q_est)

        # alpha = m1*d1**2 + I_1 + m2*(l1**2 + d2**2) + I_2
        # beta = m2*l1*d2
        # gamma = m2*d2**2 + I_2


        alpha = m1*d1**2 + I_1 + m2*(l1**2 + d2**2) + I_2 + m3*(l1**2 + l2**2) + I_3
        beta = m2*l1*d2 + m3*l1*l2
        gamma = m2*d2**2 + I_2 + m3*l2**2 + I_3

        M = []
        M.append([alpha + 2*beta*c2, gamma + beta*c2])
        M.append([gamma + beta*c2, gamma])
        M_inv = np.linalg.inv(M)
        self.set_b(M_inv[i, i])
        return u
