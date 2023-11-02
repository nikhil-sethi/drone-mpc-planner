from typing import Any
from conf import Dynamics
from numpy import cos, sin
import numpy as np

class DynamicsModel:
    """Base class for all dynamic models"""
    def __init__(self, n_states, n_actions, **kwds) -> None:
        self.N_STATES = n_states
        self.N_ACTIONS = n_actions

    def __call__(self, state, action, **kwds: Any) -> Any:
        raise NotImplementedError

class FirstOrder(DynamicsModel):
    """Constant velocity, zero acceleration"""
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
    """Acceleration control with x,y,z components"""
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

class SecondOrderRPMagGravity(SecondOrderGravity):
    """Acceleration control with Roll, Pitch and magnitude
    
    y: +ve to right
    x: +ve forward 
    z: +ve upward 
    roll: +ve CCW from the x axis 
    pitch: +ve CW from y axis
    """
    def __call__(self, state, action, **kwds: Any) -> Any:
        dx = state[3]
        dy = state[4]
        dz = state[5]
        mag = action[0]
        roll = action[1]
        pitch = action[2]

        dvx = mag * (sin(pitch) * cos(roll))
        dvy = -mag * (sin(roll))
        dvz = mag * (cos(pitch) * cos(roll)) - self.g0
        
        return [dx, dy, dz, dvx, dvy, dvz]

class FullDroneModel(DynamicsModel):
    """Base class for full drone models. Action is individual rotor RPM"""
    def __init__(self, n_states=12, n_actions=4, mass=1) -> None:
        super().__init__(n_states, n_actions)
        self.mass = mass

        # keeping these things default for now, ideally should be in args and coming from the 'agent' class that uses the model
        self.Ixx = 3.5e-5   # [kg.m^2] Inertia moment around x-axis
        self.Iyy = 3.5e-5   # [kg.m^2] Inertia moment around y-axis
        self.Izz = 6.25e-5   # [kg.m^2] Inertia moment around z-axis
        self.Cd  = 7.9379e-07 # [N/krpm^2] Drag coef
        self.Ct  = 4.000012494717929e-05    # [N/krpm^2] Thrust coef
        self.dq  = 62e-3      # [m] distance between motors' center
        self.l   = self.dq/2       # [m] distance between motors' center and the axis of rotation
        self.to_TM = np.array([[self.Ct, self.Ct*self.l, self.Ct*self.l, self.Cd]]).T * np.array([[1 ,1, 1 ,1],
                                                                                        [-1, -1, 1, 1],
                                                                                        [-1, 1, 1, -1],
                                                                                        [-1, 1, -1, 1]])

class DroneRPY(FullDroneModel):
    """Full nonlinear drone model with roll, pitch, yaw and RPM as input"""
    def __init__(self, n_states=12, n_actions=4, mass=1) -> None:
        super().__init__(n_states, n_actions, mass) 

    def __call__(self, state, action, **kwds: Any) -> Any:
        x = state[0]
        y = state[1]
        z = state[2]
        phi = state[3]
        theta = state[4] 
        psi = state[5]
        vbx = state[6]
        vby = state[7]
        vbz = state[8]
        p = state[9]
        q = state[10]
        r = state[11]

        TM = self.to_TM @ (action**2)  # thrust and momentum

        dx = vbx
        dy = vby
        dz = vbz
        dphi = p
        dtheta = q
        dpsi = r
        dvbx = (cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi))*TM[0]/self.mq
        dvby = (sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi))*TM[0]/self.mq
        dvbz = ((cos(phi)*cos(theta))*TM[0]/self.mq) - self.g0
        
        dp = ((-self.Iyy + self.Izz)*q*r + TM[1])/self.Ixx
        dq = ((self.Ixx - self.Izz)*p*r  + TM[2])/self.Iyy
        dr = ((-self.Ixx + self.Iyy)*p*q + TM[3])/self.Izz

        return [dx,dy,dz,dphi, dtheta, dpsi, dvbx,dvby,dvbz,dp,dq,dr]
    
class DroneQuat(FullDroneModel):
    def __init__(self, n_states=12, n_actions=4, mass=1) -> None:
        super().__init__(n_states, n_actions, mass)

    def __call__(self, state, action, **kwds: Any) -> Any:
        x = state[0]
        y = state[1]
        z = state[2]
        q1 = state[3]
        q2 = state[4]
        q3 = state[5]
        q4 = state[6]
        vbx = state[7]
        vby = state[8]
        vbz = state[9]
        wx = state[10]
        wy = state[11]
        wz = state[12]

        w1 = action[0]
        w2 = action[1]
        w3 = action[2]
        w4 = action[3]

        # w1, w2, w3, w4 = u

        # print
        dxq = vbx*(2*q1**2 + 2*q2**2 - 1) - vby*(2*q1*q4 - 2*q2*q3) + vbz*(2*q1*q3 + 2*q2*q4)
        dyq = vby*(2*q1**2 + 2*q3**2 - 1) + vbx*(2*q1*q4 + 2*q2*q3) - vbz*(2*q1*q2 - 2*q3*q4)
        dzq = vbz*(2*q1**2 + 2*q4**2 - 1) - vbx*(2*q1*q3 - 2*q2*q4) + vby*(2*q1*q2 + 2*q3*q4)
        dq1 = - (q2*wx)/2 - (q3*wy)/2 - (q4*wz)/2
        dq2 = (q1*wx)/2 - (q4*wy)/2 + (q3*wz)/2
        dq3 = (q4*wx)/2 + (q1*wy)/2 - (q2*wz)/2
        dq4 = (q2*wy)/2 - (q3*wx)/2 + (q1*wz)/2
        dvbx = vby*wz - vbz*wy + self.g0*(2*q1*q3 - 2*q2*q4)
        dvby = vbz*wx - vbx*wz - self.g0*(2*q1*q2 + 2*q3*q4)
        dvbz = vbx*wy - vby*wx - self.g0*(2*q1**2 + 2*q4**2 - 1) + (self.Ct*(w1**2 + w2**2 + w3**2 + w4**2))/self.mq
        dwx = -(self.Ct*self.l*(w1**2 + w2**2 - w3**2 - w4**2) - self.Iyy*wy*wz + self.Izz*wy*wz)/self.Ixx
        dwy = -(self.Ct*self.l*(w1**2 - w2**2 - w3**2 + w4**2) + self.Ixx*wx*wz - self.Izz*wx*wz)/self.Iyy
        dwz = -(self.Cd*(w1**2 - w2**2 + w3**2 - w4**2) - self.Ixx*wx*wy + self.Iyy*wx*wy)/self.Izz

        [dxq,dyq,dzq,dq1,dq2,dq3,dq4,dvbx,dvby,dvbz,dwx,dwy,dwz]