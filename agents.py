import casadi as ca
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver, AcadosOcpSolver, AcadosOcp
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import animation
from models import DynamicsModel, FirstOrder
from controllers import Constant
from casadi import sin, cos, tan
from typing import Any
from conf import Dynamics, geofence
from utils import sample_in_range, in_arena


np.set_printoptions(precision=5)

class Particle:
    def __init__(self, model:DynamicsModel, controller, n_states=6, n_actions=3, **kwds:Any) -> None:
        self.N_STATES = n_states
        self.N_ACTIONS = n_actions # motor speeds
        self.n_dims = 3
        self.dt = Dynamics.dt # timestep
        if "init_state" in kwds:
            self.init_state = np.array(kwds["init_state"])
        else:
            self.init_state = np.zeros(n_states)

        self.model = model
        self.controller = controller
        self.status = "Chillin'"
        self.reset()

    def reset(self):
        self.state = self.init_state.copy()
        self.history = {"state":[self.init_state], "action":[]}
        return self.state
    
    def update(self, **kwds):
        """Simple integration"""
        action = self.controller(self.state, **kwds)
        self.state += np.array(self.model(self.state, action, **kwds))*self.dt
        return self.state
    
    def simulate(self, Ts):
        """forward simulate system for Ts seconds"""
        self.reset()

        N = int(Ts/self.dt)
        for _ in range(N):
            self.update()
            self.history["state"].append(self.state.copy())

        return self.state

    @property
    def pos(self):
        return self.x[:self.n_dims]

    @property
    def vel(self):
        return self.x[self.n_dims:2*self.n_dims]

    @property
    def acc(self):
        return self.x[6:9]

class Drone(Particle):
    def __init__(self, model: DynamicsModel, controller, n_states=6, n_actions=3, **kwds: Any) -> None:
        super().__init__(model, controller, n_states, n_actions, **kwds)
        self.l = 0.05
        self.motor_pos = np.array([
            [self.l, -self.l, 0],
            [-self.l, -self.l, 0],
            [-self.l, self.l, 0],
            [self.l, self.l, 0],
        ])
        self.status = "Killin'"


class Crazyflie(Drone):
    def __init__(self) -> None:
        super().__init__()

class PATSX(Particle):
    def __init__(self) -> None:
        super().__init__(n_states=6, n_actions=3)
        self.mq = 0.0306
        self.hov_T = self.g0

        self.x0 = np.zeros(self.N_STATES)
        self.x0[1] = -0.99 # starts at 0.5 meters below the camera
        self.x0[2] = -0.99
        self.x0[-3:] = np.array([0,0,0])

    def xdot_trp(self, _x, _u):
        dx = _x[3]
        dy = _x[4]
        dz = _x[5]
        dvx = _u[0] * (sin(_u[2]) * cos(_u[1]))
        dvy = -_u[0] * (sin(_u[1]))
        dvz = _u[0] * (cos(_u[2]) * cos(_u[1])) - self.g0
        
        return [dx, dy, dz, dvx, dvy, dvz]

class Moth(Particle):
    moth_vel = [1.5,0,0]
    MAX_VEL = 2
    def __init__(self, n_states=6, n_actions=3, **kwds: Any) -> None:
        model = FirstOrder(n_states=n_states, n_actions=n_actions)
        if 'velocity' in kwds:
            velocity = kwds["velocity"]
        else:
            velocity = Moth.moth_vel
        
        # for a constant controller, just use the initial velocity
        controller = Constant(action=np.array(kwds["init_state"][-3:]))
        super().__init__(model, controller, n_states, n_actions, **kwds)

    def randomize(self):
        """Random search until you find an interception position within the geofence"""
        
        self.init_state[:3] = sample_in_range(np.array(geofence).T) # position
        self.init_state[3:] = Moth.MAX_VEL*(-1+2*np.random.rand(3)) # velocity
        
        max_pos = self.simulate(1.8) # simulate for maximum interception time and do arena check
        if not in_arena(max_pos): # if not in arena, reset again
            self.randomize()
        self.controller.action = self.init_state[3:]
        return super().reset()
