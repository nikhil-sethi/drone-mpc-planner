import casadi as ca
from acados_template import AcadosModel, AcadosSim, AcadosSimSolver, AcadosOcpSolver, AcadosOcp
import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib import animation
import scipy
from casadi import sin, cos, tan

np.set_printoptions(precision=5)

class Particle:
    def __init__(self) -> None:
        self.N_STATES = 4
        self.N_ACTIONS = 2 # motor speeds
        self.m = 1
        self.g0  = 9.81     # [m.s^2] accerelation of gravity
        self.dt = 0.035 # timestep

    def xdot(self, x,u):

        # position derivative
        dx = x[3]
        dy = x[4]
        dz = x[5]

        # velocity derivative
        dvx = u[0]
        dvy = u[1]
        dvz = u[2] - self.g0

        return [dx, dy, dz, dvx, dvy, dvz]

class Drone(Particle):
    def __init__(self) -> None:
        super().__init__()

        # parameters: # taken from webots mass tab
       
        self.mq  = 0.05     # [kg] total mass (with one marker)
    
        self.Ixx = 3.5e-5   # [kg.m^2] Inertia moment around x-axis
        self.Iyy = 3.5e-5   # [kg.m^2] Inertia moment around y-axis
        self.Izz = 6.25e-5   # [kg.m^2] Inertia moment around z-axis
        self.Cd  = 7.9379e-07 # [N/krpm^2] Drag coef
        self.Ct  = 4.000012494717929e-05    # [N/krpm^2] Thrust coef
        self.dq  = 62e-3      # [m] distance between motors' center
        self.l   = self.dq/2       # [m] distance between motors' center and the axis of rotation
        self.N_STATES = 12
        self.N_ACTIONS = 4 # motor speeds

        self.dx = np.zeros(self.N_STATES)
        
        self.hov_w = np.sqrt(self.mq*self.g0/4/self.Ct)

        # linear roll pitch model
        self.A = np.zeros((self.N_STATES, self.N_STATES))
        self.A[:self.N_STATES//2, self.N_STATES//2:] = np.eye(self.N_STATES//2)
        self.A[7,3] = -self.g0
        self.A[6,4] = self.g0

        self.B = np.zeros((self.N_STATES, self.N_ACTIONS))
        self.B[8] = (2*self.Ct/self.mq) * np.array([1,1,1,1])
        self.B[11] = (2*self.Cd/self.Izz) * np.array([-1,1,-1,1])
        self.B[10] = (self.dq*self.Ct/self.Iyy) * np.array([-1,1,1,-1])
        self.B[9] = (self.dq*self.Ct/self.Ixx) * np.array([-1,-1,1,1])

        self.B *= self.hov_w
        print(self.A)
        print(self.B)

        self.motor_pos = np.array([
            [self.l, -self.l, 0],
            [-self.l, -self.l, 0],
            [-self.l, self.l, 0],
            [self.l, self.l, 0],
        ])

        self.to_TM = np.array([[self.Ct, self.Ct*self.l, self.Ct*self.l, self.Cd]]).T * np.array([[1 ,1, 1 ,1],
                                                                                              [-1, -1, 1, 1],
                                                                                              [-1, 1, 1, -1],
                                                                                              [-1, 1, -1, 1]])

    def xdot(self, _x, u):
    
        # x,y,z,q1, q2, q3, q4, vbx, vby, vbz, wx, wy, wz = x
        # a = x[0]
        x = _x[0]
        y = _x[1]
        z = _x[2]
        q1 = _x[3]
        q2 = _x[4]
        q3 = _x[5]
        q4 = _x[6]
        vbx = _x[7]
        vby = _x[8]
        vbz = _x[9]
        wx = _x[10]
        wy = _x[11]
        wz = _x[12]

        w1 = u[0]
        w2 = u[1]
        w3 = u[2]
        w4 = u[3]

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

        # print(dxq,dyq,dzq,dq1, dq2,dq3,dq4,dvbx,dvby,dvbz,dwx,dwy,dwz)
        # self.dx = cp.hstack([dxq,dyq,dzq,dq1,dq2,dq3,dq4,dvbx,dvby,dvbz,dwx,dwy,dwz])
        # print(self.dx)
        # x_next = x + self.dx*self.dt

        # x += dxq*self.dt
        # y += dyq*self.dt
        # z += dzq*self.dt
        # q1 += dq1*self.dt
        # q2 += dq2*self.dt
        # q3 += dq3*self.dt
        # q4 += dq4*self.dt
        # vbx += dvbx*self.dt
        # vby += dvby*self.dt
        # vbz += dvbz*self.dt
        # wx += dwx*self.dt
        # wy += dwy*self.dt
        # wz += dwz*self.dt
        
        # print(x_next)
        return [dxq,dyq,dzq,dq1,dq2,dq3,dq4,dvbx,dvby,dvbz,dwx,dwy,dwz]

    def xdot_rpy(self, _x, u):
        x = _x[0]
        y = _x[1]
        z = _x[2]
        phi = _x[3]
        theta = _x[4] 
        psi = _x[5]
        vbx = _x[6]
        vby = _x[7]
        vbz = _x[8]
        p = _x[9]
        q = _x[10]
        r = _x[11]

        TM = self.to_TM @ (u**2)  # thrust and momentum

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

    def xdot_linear(self, _x, u):
        return self.A @ _x + self.B @ u

class Crazyflie(Drone):
    def __init__(self) -> None:
        super().__init__()

class PATSX(Particle):
    def __init__(self) -> None:
        super().__init__()
        self.N_STATES = 6   
        self.N_ACTIONS = 3 #
        self.mq = 0.0306
        self.hov_T = self.g0

        self.x0 = np.zeros(self.N_STATES)
        
    def xdot_trp(self, _x, _u):
        dx = _x[3]
        dy = _x[4]
        dz = _x[5]
        dvx = _u[0] * (sin(_u[2]) * cos(_u[1]))
        dvy = -_u[0] * (sin(_u[1]))
        dvz = _u[0] * (cos(_u[2]) * cos(_u[1])) - self.g0
        
        return [dx, dy, dz, dvx, dvy, dvz]
