import numpy as np
from .controller import Controller
from models.mmac_1 import ManiuplatorModel1 as ManipulatorModel1
from models.mmac_2 import ManiuplatorModel2 as ManipulatorModel2
from models.mmac_3 import ManiuplatorModel3 as ManipulatorModel3


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.Tp = Tp
        self.models = [ManipulatorModel1(Tp), ManipulatorModel2(Tp), ManipulatorModel3(Tp)]
        self.i = 0
        self.prev_i = self.i
        self.errors = []
        self.prev_u = [0.0, 0.0]
        self.prev_x = [0.0, 0.0, 0.0, 0.0]
        self.first = True

    def choose_model(self, x):
        self.errors = [0, 0, 0]

        for i in range(0,3):
            M = self.models[i].M(self.prev_x)
            C = self.models[i].C(self.prev_x)
            q_ddot = np.linalg.solve(M, self.prev_u - np.dot(C, self.prev_x[2:]))

            q_dot = self.prev_x[2:] + q_ddot * self.Tp
            q = x[:2] + q_dot * self.Tp

            # self.states[i] = self.states[i] + res
            error = abs(q[0] - x[0]) + abs(q[1] - x[1]) + abs(q_dot[0] - x[2]) + abs(q_dot[1] - x[3])
            self.errors[i] = error

            # self.errors[i].append(np.sum(np.abs(e)))

        # sum_errors = [np.sum(self.errors[i]) for i in range(3)]
        newIndex = np.argmin(self.errors)
        self.i = newIndex
        if self.i != self.prev_i:
            self.prev_i = self.i
            print("Switched to model", self.i)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        if self.first:
            self.first = False
            self.x_prev = x
        Kd = 1
        Kp = 1
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        v = q_r_ddot + Kd * (q_r_dot - q_dot) + Kp * (q_r - q)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v + C @ q_dot
        self.prev_u = u
        self.prev_x = x
        return u
