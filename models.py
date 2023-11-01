from typing import Any
from conf import Dynamics

class DynamicsModel:
    def __init__(self, n_states, n_actions) -> None:
        self.N_STATES = n_states
        self.N_ACTIONS = n_actions

    def __call__(self, state, action, **kwds: Any) -> Any:
        raise NotImplementedError

class FirstOrder(DynamicsModel):
    def __init__(self, n_states, n_actions) -> None:
        super().__init__(n_states, n_actions)

    def __call__(self, state, action, **kwds: Any) -> Any:
        dx = action[0]
        dy = action[1]
        dz = action[2]
        du = 0
        dv = 0
        dw = 0
        return [dx, dy, dz, du, dv, dw]

class SecondOrderGravity(DynamicsModel):
    def __init__(self, n_states, n_actions, mass = 1, gravity = [0,-Dynamics.G, 0]) -> None:
        super().__init__(n_states, n_actions)
        self.mass = mass
        self.gravity = gravity

    def __call__(self, state, action, **kwds: Any) -> Any:
        # position derivative
        dx = state[3]
        dy = state[4]
        dz = state[5]

        # velocity derivative
        dvx = (action[0] + self.gravity[0])/self.mass
        dvy = (action[1] + self.gravity[1])/self.mass
        dvz = (action[2] + self.gravity[2])/self.mass

        return [dx, dy, dz, dvx, dvy, dvz]
    