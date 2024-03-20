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
        self.models = [ManipulatorModel1(Tp), ManipulatorModel2(Tp), ManipulatorModel3(Tp)]
        self.i = 0

    def choose_model(self, x):
        q = x[:2]
        q_dot = x[2:]
        errors = []

        for i in range(3):
            M = self.models[i].M(x)
            C = self.models[i].C(x)
            e = M @ q_dot[:, np.newaxis] + C @ q_dot[:, np.newaxis]
            errors.append(e)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        v = q_r_ddot # TODO: add feedback
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        return u
