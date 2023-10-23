import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from cvxopt import solvers, matrix
from itertools import product
import scipy
from matplotlib.backend_bases import MouseButton
from matplotlib import patches
from matplotlib.widgets import Slider
import time
import enum

class LogLevel(enum.IntEnum):
    NONE = 0
    INFO = 1
    DEBUG = 2
    ERROR = 3


AMAX = 20
VMAX = 4


def fact(j, i):
    """factorial of j upto i. Used for derivatives"""
    if i==0:
        return 1
    return j*fact(j-1, i-1)

def pow(n, k):
    if k<0:
        return 1
    return n**k

np.set_printoptions(precision=4, suppress=True)
np.set_printoptions(threshold=np. inf, suppress=True, linewidth=np. inf)
class TrajectoryManager():
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO) -> None:
        
        self.wps = waypoints
        
        self.l  = len(waypoints) # number of dimensions
        
        self.m = len(waypoints[0])-1 # number of segments
        
        assert self.m >=1, "There must be at least two waypoints for a path"
        self.n = order # order of control/number of derivatives
        assert 0<self.n<=4, "Order of polynomial must be between 0 and 4"
        
        if type(time) is int or type(time) is float:
            self.t = np.cumsum(self.get_path_wts()*time) # cumulative times for waypoints 
            self.t = np.insert(self.t, 0,0)
            
            # time mutation
            # self.t[1]-=0.1
            # self.t[-2]-=0.2
        elif type(time) is list:
            assert len(time) == self.m+1, "Number of timestamps must be equal to the number of waypoints."
            self.t = time
        self.log_level = log_level

    def set_segment_times(self, times):
        """Set externally from an outer optimizer"""
        self.t = times

    def set_waypoints(self, wps):
        """Set externally from an outer optimizer"""
        self.wps = wps

    def get_path_wts(self):
        """returns distance based weights of the path"""
        wps = np.array(self.wps)
        diffs = np.linalg.norm(wps[:,1:] - wps[:,:-1], axis=0)
        return diffs/sum(diffs)

    def generate(self, coeffs_raw, order = 0, dt=0.05):
        """works only for two segments, some hardcoding but fast"""
        C = np.array(coeffs_raw).reshape(self.l, self.m, 2*self.n) # coefficients
        self.tvec = np.arange(0, sum(self.t)+dt, dt)

        diff = [fact(j,order) for j in range(2*self.n)]
        diff = [fact(j,order) for j in range(2*self.n)]
        
        diff = [fact(j,order) for j in range(2*self.n)]        
        
        time_powers = np.concatenate([[0]*order, np.arange(2*self.n -order)])
        pvec = diff*(np.tile(self.tvec[:, np.newaxis], 2*self.n) ** time_powers) # (d/l x order)
        c_mask = np.zeros_like(self.tvec)
        for t in self.t[1:-1]:
            c_mask += (self.tvec>=t).astype(int)

        traj = np.sum(C[:, c_mask.astype(int)]*pvec, axis=2).T

        return traj
        
    def plot(self, points, ax=None)	:
        _, dims = points.shape
        if dims == 3: # 3D
            # self.ax = plt.axes(projection='3d')
            # ax.set_xlim(-1.5,1.5)
            # ax.set_ylim(-1.5,1.5)
            # ax.set_zlim(-1.5,1.5)
            ax.plot(self.wps[0],self.wps[2],self.wps[1],'b--')
            ax.plot(points[:,0],points[:,2],points[:,1], 'r-', linewidth=3)
        elif dims == 2:
            # ax.plot(self.wps[0],self.wps[1],'k--', linewidth=2)
            ax.plot(points[:,0],points[:,1], 'b-', linewidth=3)
            ax.plot(self.wps[0],self.wps[1],'ro')
        elif dims == 1:
            """plot against time"""
            pass
        #plt.show()

    def plot_vec(self, pos, vec, ax=None, color='k'):
        _, dims = pos.shape
        if dims == 3: # 3D
            ax.quiver(pos[::2,0], 0, pos[::2,1], vec[::2,0], 0, vec[::2,1], color=color, arrow_length_ratio=0.1)
        else:
            ax.quiver(pos[:,0], pos[:,1], vec[:,0],vec[:,1], color=color, width=0.005)
        

class MinVelAccJerkSnapCrackPop(TrajectoryManager): # cute name
    def __init__(self, order, waypoints, time = 1, log_level=LogLevel.INFO, a_t = None) -> None:
        # calc 'moving forward' bounds for theta opt
        vec_incoming = waypoints[1] - waypoints[0]
        theta_incoming = np.arctan2(vec_incoming[1], vec_incoming[0])
        
        # acc_theta_bounds = (, (theta_incoming+np.pi/4)/(2*np.pi))
        ic_theta = (theta_incoming - np.pi/2) + (np.pi) *(a_t[0])
        # print(theta_incoming, ic_theta)
        ic_mag = 5 + 10*a_t[1]
        
        self.a_t = ic_mag*np.array([np.cos(ic_theta), np.sin(ic_theta)])

        super().__init__(order, waypoints, time, log_level)
        
        # setup constraints
        if self.log_level>=LogLevel.INFO:
            print("[Optimizer] Setting up constraints")
        self.setup_constraints()

        # setup objectives
        if self.log_level >=LogLevel.INFO:
            print("[Optimizer] Setting up optimization objective")
        self.setup_objectives()

    def setup_constraints(self):
        # Endpoint constraints
        A_ep = []

        # CLEAN THIS SHIT UP
        # first waypoint
        m=0
        for n in range(self.n):
            A_ep_i = [0]*2*self.n*m
            A_ep_i.extend([fact(j,n)*pow(self.t[m], j-n) for j in range(2*self.n)])	
            A_ep_i.extend([0]*2*self.n*(self.m-m-1))
            A_ep.append(A_ep_i)

        # last waypoint
        m = self.m
        for n in range(1,self.n):
            A_ep_i = [0]*2*self.n*(m-1)
            A_ep_i.extend([fact(j,n)*pow(self.t[m], j-n) for j in range(2*self.n)])	
            A_ep_i.extend([0]*2*self.n*(self.m-m))
            A_ep.append(A_ep_i)

        # Continuity constraints
        A_con = []
        for m in range(1,self.m):
            for n in range(self.n+1):
                tpoly = [fact(j,n)*pow(self.t[m], j-n) for j in range(2*self.n)]
                if n==0:
                    A_con_i = [0]*2*self.n*(m-1)
                    # add 2 position constraints at same time index
                    A_con_i.extend(tpoly + [0]*2*self.n)	# end position of prev poly
                    A_con_i.extend([0]*2*self.n*(self.m-m-1))
                    A_con.append(A_con_i)

                    A_con_i = [0]*2*self.n*(m-1)
                    # add 2 position constraints at same time index
                    A_con_i.extend([0]*2*self.n + tpoly)	# end position of prev poly
                    A_con_i.extend([0]*2*self.n*(self.m-m-1))
                    A_con.append(A_con_i)

                else: # add self.n-1 continuity constraints
                    A_con_i = [0]*2*self.n*(m-1)
                    A_con_i.extend(tpoly + [-a for a in tpoly])
                    A_con_i.extend([0]*2*self.n*(self.m-m-1))
                    A_con.append(A_con_i)

        # combine endpoint and continuity constraints
        self.A = A_ep + A_con

    def setup_objectives(self):
        self.H = np.zeros((2*self.n*self.m, 2*self.n*self.m))
        for m in range(self.m):
            # only n coefficients out of 2n will be active. Need to set H accordingly
            self.H[2*self.n*m:2*self.n*(m+1), 2*self.n*m:2*self.n*(m+1)] = self.get_H_1seg(self.t[m+1])

        # self.f = np.zeros(2*self.n*self.m)

        # self.f = np.zeros(self.H.shape[0])
        a_t_x = AMAX*np.cos(self.theta_t)
        a_t_y = AMAX*np.sin(self.theta_t)
        self.a_t = np.array([a_t_x, a_t_y])
        if self.log_level >= LogLevel.DEBUG:
            print("Target acc. at interception: ", a_t_x, a_t_y)
        t1 = self.t[1]
        self.f = np.array([
            0,0,0,0,0,0,
            0, 0, -4*a_t_x, -12*t1*a_t_x, -24*a_t_x*t1**2, -40*a_t_x*t1**3
            ])


    def get_H_1seg(self, T):
        """Quadratic Cost matrix for integration of a squared n degree polynomial"""
        H_seg = np.zeros((2*self.n, 2*self.n))

        # The derivative coeffecients that come about after differentiating the polynomial n times
        diff = [fact(j,self.n) for j in range(self.n, 2*self.n)]
        # When sqaured and integrated the polymial will be quadratic and will have pairwise permutations of these coefficients
        # These coefficients will come up in the matrix afterwards 
        coeff_prods = np.prod(list(product(diff,repeat=2)),-1).reshape(self.n,self.n)

        # the powers to which the time endpoint will be raised. This comes from the result of integrating the sqaure of the
        # n times differentiated polynomial.
        time_powers = np.sum(np.meshgrid(np.arange(self.n), np.arange(self.n)),0)+1 
        time_poly = T**time_powers/time_powers # this result comes because of 2n-1 order integration
        H_seg[self.n:,self.n:] = coeff_prods*time_poly
        
        return H_seg

    def optimize(self, plan_order = 0, num_pts=150):
        """
        Return an optimized plan for all dimensions
        """
        print("[Optimizer] Starting optimization...")
        coeffs = []
        for l in range(self.l):
            b_ep = [self.wps[l][0]] + [0]*(self.n-1) + [0]*(self.n-1)
            b_con = []
            for i in range(1, self.m):
                b_con += [self.wps[l][i]]*2 + [0]*self.n 
            b = b_ep + b_con	# continuity RHS
            # print(np.array(self.A))
            self.sol = solvers.qp(P = matrix(self.H), q=matrix(self.f, tc='d'), A=matrix(np.array(self.A), tc='d'), b=matrix(b, tc='d'))
            coeffs.extend(self.sol["x"])

        plan = self.generate(coeffs, order = plan_order, dt=0.05)
        print("[Optimizer] Done.")
        return np.array(plan).T

class MinVelAccJerkSnapCrackPopCorridor(MinVelAccJerkSnapCrackPop):
    def __init__(self, order, waypoints, time=1) -> None:
        # discretize waypoints
        num = 2
        temp = np.linspace(waypoints[:,:-1], waypoints[:,1:], num=num, axis=-1, endpoint=False).reshape(waypoints.shape[0], (waypoints.shape[1]-1)*num)
        waypoints = np.append(temp, waypoints[:,-1][:,None], axis=1)
        
        super().__init__(order, waypoints, time)

class MinAcc2WP2D(MinVelAccJerkSnapCrackPop):
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO, a_t=None) -> None:
        super().__init__(order, waypoints, time, log_level, a_t=a_t)
        
        # print(np.linalg.matrix_rank(self.H), np.linalg.matrix_rank(self.A))
        
    def setup_constraints(self):
        t0 = self.t[0]
        t1 = self.t[1]
        t2 = self.t[2] +t1
        self.A = np.array([
            ## X
            # === endpoint constraints ===
            # position
            [t0**0, t0**1, t0**2, t0**3, 0, 0, 0, 0,                 0, 0, 0, 0, 0, 0, 0, 0], #  start position
            # [0, 0, 0, 0,                 t2**0, t2**1, t2**2, t2**3, 0, 0, 0, 0, 0, 0, 0, 0], #  end position
            # [-1, -(1+t1), -(2*t1 + t1**2), -(3*t1**2 + t1**3),   t2**0, t2**1, t2**2, t2**3,  0, 0, 0, 0, 0, 0, 0, 0], 
            # velocity
            [0, t0**0, 2*t0**1, 3*t0**2, 0, 0, 0, 0,                 0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            [0, 0, 0, 0,                 0, t2**0, 2*t2**1, 3*t2**2, 0, 0, 0, 0, 0, 0, 0, 0], #  end velocity
            # acceleration
            # [0, 0, 2*t0**0, 6*t0**1, 0, 0, 0, 0,             0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            # [0, 0, 0, 0,             0, 0, 2*t1**0, 6*t1**1, 0, 0, 0, 0, 0, 0, 0, 0], #  end velocity
            # [0, 0, 2*t1**0, 6*t1**1, 0, 0, 0, 0,             0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            # [0, 0, 0, 0,             0, 0, 2*t2**0, 6*t2**2, 0, 0, 0, 0, 0, 0, 0, 0], #  start velocity

            # ===== continuity constraints =====
            [0, 0, 0, 0,                 t1**0, t1**1, t1**2, t1**3, 0, 0, 0, 0, 0, 0, 0, 0], 
            [t1**0, t1**1, t1**2, t1**3, 0, 0, 0, 0,                 0, 0, 0, 0, 0, 0, 0, 0], 
            # [-1, -(1+t1), -(2*t1 + t1**2), -(3*t1**2 + t1**3),   t2**0, t2**1, t2**2, t2**3,  0, 0, 0, 0, 0, 0, 0, 0], 
            [0, t1**0, 2*t1**1, 3*t1**2,  -0, -t1**0, -2*t1**1, -3*t1**2,  0, 0, 0, 0, 0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0,     2*t1**0, 6*t1**1,  -0, -0,     -2*t1**0, -6*t1**1,  0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point

            ## Y
            # === endpoint constraints ===
            # position
            [0, 0, 0, 0,  0, 0, 0, 0,  t0**0, t0**1, t0**2, t0**3, 0, 0, 0, 0,                ], #  start position
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,                 t2**0, t2**1, t2**2, t2**3,], #  end position
            # [0, 0, 0, 0, 0, 0, 0, 0, -1, -(1+t1), -(2*t1 + t1**2), -(3*t1**2 + t1**3),   t2**0, t2**1, t2**2, t2**3], 
            # velocity
            [0, 0, 0, 0,  0, 0, 0, 0,  0, t0**0, 2*t0**1, 3*t0**2, 0, 0, 0, 0,                 ], #  start velocity
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,                 0, t2**0, 2*t2**1, 3*t2**2, ], #  end velocity
            # acceleration
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 2*t0**0, 6*t0**1,     0, 0, 0, 0,            ], #  start velocity
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,                 0, 0, 2*t1**0, 6*t1**1,], #  end velocity
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 2*t1**0, 6*t1**1,     0, 0, 0, 0,            ], #  start velocity
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,                 0, 0, 2*t2**0, 6*t2**2,], #  start velocity

            # ===== continuity constraints ====
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,                 t1**0, t1**1, t1**2, t1**3,], #  end position
            [0, 0, 0, 0,  0, 0, 0, 0,  t1**0, t1**1, t1**2, t1**3, 0, 0, 0, 0,                ], #  start position
            # [0, 0, 0, 0, 0, 0, 0, 0, -1, -(1+t1), -(2*t1 + t1**2), -(3*t1**2 + t1**3),   t2**0, t2**1, t2**2, t2**3], 
            [0, 0, 0, 0,  0, 0, 0, 0,  0, t1**0, 2*t1**1, 3*t1**2,  -0, -t1**0, -2*t1**1, -3*t1**2,  ], #  velocity continuity at mid point
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 2*t1**0, 6*t1**1,      -0, -0, -2*t1**0, -6*t1**1,      ] #  acceleration continuity at mid point
        ])

        amax=AMAX

        # self.b = np.array([
        #     self.wps[0][0], self.wps[0][2], 0,0,    self.wps[0][1], self.wps[0][1], 0,0,
        #     self.wps[1][0], self.wps[1][2], 0,0,    self.wps[1][1], self.wps[1][1], 0,0,
        # ])

        self.b = np.array([
            self.wps[0][0], 0,0,    self.wps[0][1], self.wps[0][1], 0,0,
            self.wps[1][0], 0,0,    self.wps[1][1], self.wps[1][1], 0,0,
        ])

        self.G = np.array([

            # Position endpoint within geofence
            [0, 0, 0, 0,                t2**0, t2**1, t2**2, t2**3, 0, 0, 0, 0, 0, 0, 0, 0], #  end X position
            [0, 0, 0, 0,             0, 0, 0, 0,                 0, 0, 0, 0, t2**0, t2**1, t2**2, t2**3], #  end Y position
            [0, 0, 0, 0,             -t2**0, -t2**1, -t2**2, -t2**3, 0, 0, 0, 0, 0, 0, 0, 0], #  end X position
            [0, 0, 0, 0,             0, 0, 0, 0,                 0, 0, 0, 0, -t2**0, -t2**1, -t2**2, -t2**3,], #  end Y position

            # --- wp 1 acceleration box ---
            [-0, -0,  0, 0,            0, 0, 2*t1**0, 6*t1**1,    0, 0, 0, 0,             0, 0, 0, 0], #  acceleration continuity at mid point
            [ -0, -0,  0, 0,           0, 0, -2*t1**0, -6*t1**1,  0, 0, 0, 0,             0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0,              -0, -0, 0, 0,              0, 0, 0, 0,           0, 0, 2*t1**0, 6*t1**1,], #  acceleration continuity at mid point
            [0, 0, 0, 0,              -0, -0, 0, 0,             0, 0, 0, 0,            0, 0, -2*t1**0, -6*t1**1,], #  acceleration continuity at mid point

            # wp 0 acceleration box
            [0, 0, 2*t0**0, 6*t0**1,  -0, -0,  0, 0,            0, 0, 0, 0,             0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, -2*t0**0, -6*t0**1,  -0, -0,  0, 0,            0, 0, 0, 0,             0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0,              -0, -0, 0, 0,             0, 0, 2*t0**0, 6*t0**1, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0,              -0, -0, 0, 0,             0, 0, -2*t0**0, -6*t0**1, 0, 0, 0, 0], #  acceleration continuity at mid point
            
            # wp 2 acceleration box
            [-0, -0,  0, 0,            0, 0, 2*t2**0, 6*t2**1,    0, 0, 0, 0,             0, 0, 0, 0], #  acceleration continuity at mid point
            [ -0, -0,  0, 0,           0, 0, -2*t2**0, -6*t2**1,  0, 0, 0, 0,             0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0,              -0, -0, 0, 0,              0, 0, 0, 0,           0, 0, 2*t2**0, 6*t2**1,], #  acceleration continuity at mid point
            [0, 0, 0, 0,              -0, -0, 0, 0,             0, 0, 0, 0,            0, 0, -2*t2**0, -6*t2**1,], #  acceleration continuity at mid point
            
            # [0, 0, 0, 0,              -0, -0, 0, 0,             0, 0, 0, 0,             0, 0, -2*t1**0, -6*t1**1,], #  acceleration continuity at mid point

        ])

        self.h = np.array([
            1,2,1,0,
            amax, amax,amax, amax, 
            amax, amax, amax, amax, 
            amax, amax, amax, amax, 
            #amax, amax,amax, amax
        ])

    def get_acc_H(self, t):
        """acceleration squared and integrated"""
        return 2*np.array([
            [0, 0,  0,  0],
            [0, 0,  0,  0],
            [0, 0,  4*t,  6*(t**2)],
            [0, 0,  6*(t**2),  12*(t**3)]
        ])
    
    def get_vel_H(self,t):
        """velocity squared"""
        return 1.5*np.array([
            [0, 0,         0,          0],
            [0, 1,         2*(t**1),  3*(t**2)],
            [0, 2*(t**1),  4*(t**2),  6*(t**3)],
            [0, 3*(t**2),  6*(t**3),  9*(t**4)]
        ])
         
    def setup_objectives(self):
        t1 = self.t[1]
        # print(t1)
        t2 = self.t[2] 
        
        # self.H = scipy.linalg.block_diag(self.get_acc_H(t1), self.get_acc_H(t2), self.get_acc_H(t1), self.get_acc_H(t2))
        self.H = scipy.linalg.block_diag(0.01*self.get_acc_H(t1), self.get_acc(t1), 0.01*self.get_acc_H(t1), self.get_acc(t1))
        # self.H = scipy.linalg.block_diag(self.get_acc_H(t1), self.get_acc_H(t2)-self.get_acc(t1), self.get_acc_H(t1), self.get_acc_H(t2)-self.get_acc(t1))

        self.f = np.zeros(self.H.shape[0])
        a_t_x = 20
        a_t_y = -20
        self.f = np.array([
            0,0,0,0,0,0,
            0, 0, 4*a_t_x, 12*t1*a_t_x, 24*a_t_x*t1**2, 40*a_t_x*t1**3,
            0,0,0,0,0,0,
            0, 0, 4*a_t_y, 12*a_t_y*t1, 24*a_t_y*t1**2, 40*a_t_y*t1**3,
            ])
        
    def optimize(self):
        """
        Return an optimized plan for all dimensions
        """
        if self.log_level >= LogLevel.INFO:
            print("[Optimizer] Starting optimization...")
        options = {"show_progress":self.log_level>=LogLevel.DEBUG}
        
        self.sol = solvers.qp(G=matrix(self.G, tc='d'), h=matrix(self.h, tc='d'),P = matrix(self.H), q=matrix(self.f), A=matrix(np.array(self.A), tc='d'), b=matrix(self.b, tc='d'), options=options)
        # print("slack: ", self.sol['s'])
        self.status = self.sol['status']
        if self.status == 'optimal':
            self.cost = self.sol['primal objective']
            pos = self.generate(self.sol['x'], dt=0.1, order=0) # coarse discretization is fine 
            acc = self.generate(self.sol['x'], dt=0.05, order=2) # need finer discretization because acc is more senstive

            # penalize geofence //  try to add this in linear quadratic program 
            if np.any(pos > np.array([1,2])+0.1) or np.any(pos<np.array([-1,0])-0.1):
                self.cost += 100000

            # penalise ovreall acceleration magnitude // is nonlinear
            if np.max(np.linalg.norm(acc, axis=1))>AMAX:
                self.cost += 100000
            
        elif self.status == 'unknown':
            self.cost = 100000
        if self.log_level >= LogLevel.INFO:
            print("[Optimizer] Done.")

        return self.cost

    def generate(self, coeffs_raw, dt, order=0):
        """works only for two segments, some hardcoding but fast"""
        C = np.array(coeffs_raw).reshape(self.l, self.m, 2*self.n) # coefficients
        self.tvec = np.arange(0,sum(self.t)+dt, dt)
        self.tvec[-1] = sum(self.t) # last point is missed out when doing np arange

        diff = [fact(j,order) for j in range(2*self.n)]        
        time_powers = np.concatenate([[0]*order, np.arange(2*self.n -order)])
        pvec = diff*(np.tile(self.tvec[:, np.newaxis], 2*self.n) ** time_powers) # (d/l x order)

        c_mask = (self.tvec>=self.t[1]).astype(int)
        # print(self.t, c_mask)
        traj = np.sum(C[:,c_mask.astype(int)]*pvec, axis=2).T

        return traj
    
    def evaluate(self, dt, order=0):
        """get a specific plan for position, velocity etc."""
        if not self.sol["x"] and self.log_level>=LogLevel.INFO:
            print("Run the optimization first")
            return
        else:
            soln = self.sol["x"]
        plan = self.generate(soln, dt=dt, order = order)
        return plan

class MinAcc3WP2D(MinAcc2WP2D):
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO) -> None:
        # time = [t[0], 0.1, t[1]]
        super().__init__(order, waypoints, time, log_level)
        
        # print(np.linalg.matrix_rank(self.H), np.linalg.matrix_rank(self.A))
        
    def setup_constraints(self):
        t0 = self.t[0]
        t1 = self.t[1]
        t2 = self.t[2] + t1
        t3 = self.t[3] + t2

        self.A = np.array([
            ## X
            # === endpoint constraints ===
            # position  
            [t0**0, t0**1, t0**2, t0**3, 0, 0, 0, 0,   0, 0, 0, 0,                     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  start position
            [0, t0**0, 2*t0**1, 3*t0**2, 0, 0, 0, 0,   0, 0, 0, 0,                     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  start velocity
            # [0, 0,     2*t0**0, 6*t0**1, 0, 0, 0, 0,   0, 0, 0, 0,                     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  start acc
            [0, 0, 0, 0,                 0, 0, 0, 0,   0, t3**0, 2*t3**1, 3*t3**2,     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  end velocity
            # [0, 0, 0, 0,                 0, 0, 0, 0,   0, 0,     2*t3**0, 6*t3**1,     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  end acc

            # ===== continuity constraints =====
            [0, 0, 0, 0,                   t1**0, t1**1, t1**2, t1**3,     0, 0, 0, 0,                       0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], 
            [t1**0, t1**1, t1**2, t1**3,   0, 0, 0, 0,                     0, 0, 0, 0,                       0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], 
            [0, 0, 0, 0,                   0, 0, 0, 0,                     t2**0, t2**1, t2**2, t2**3,       0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], 
            [0, 0, 0, 0,                   t2**0, t2**1, t2**2, t2**3,     0, 0, 0, 0,                       0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], 
            [0, t1**0, 2*t1**1, 3*t1**2,  -0, -t1**0, -2*t1**1, -3*t1**2,  0, 0, 0, 0,                       0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0,     2*t1**0, 6*t1**1,  -0, -0,     -2*t1**0, -6*t1**1,  0, 0, 0, 0,                       0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0,                   0, t2**0, 2*t2**1, 3*t2**2,    -0, -t2**0, -2*t2**1, -3*t2**2,    0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0, 0, 0,                   0, 0,     2*t2**0, 6*t2**1,    -0, -0,     -2*t2**0, -6*t2**1,   0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0], #  acceleration continuity at mid point
            
            
            ## Y
            # === endpoint constraints ===
            # position
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,      t0**0, t0**1, t0**2, t0**3, 0, 0, 0, 0,  0, 0, 0, 0                 ], #  start position
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,      0, t0**0, 2*t0**1, 3*t0**2, 0, 0, 0, 0,  0, 0, 0, 0                 ], #  start velocity
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,      0, 0,     2*t0**0, 6*t0**1, 0, 0, 0, 0,  0, 0, 0, 0                 ], #  start acc
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,      0, 0, 0, 0,                 0, 0, 0, 0,  0, t3**0, 2*t3**1, 3*t3**2 ], #  end velocity
            # [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,      0, 0, 0, 0,                 0, 0, 0, 0,  0, 0,     2*t3**0, 6*t3**1 ], #  end acc

            # ===== continuity constraints =====
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0,                 t1**0, t1**1, t1**2, t1**3,    0, 0, 0, 0], 
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    t1**0, t1**1, t1**2, t1**3, 0, 0, 0, 0,                    0, 0, 0, 0], 
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0,    0, 0, 0, 0,         t2**0, t2**1, t2**2, t2**3], 
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0,    t2**0, t2**1, t2**2, t2**3,             0, 0, 0, 0], 
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, t1**0, 2*t1**1, 3*t1**2,  -0, -t1**0, -2*t1**1, -3*t1**2,  0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0,     2*t1**0, 6*t1**1,  -0, -0,     -2*t1**0, -6*t1**1,  0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0,    0, t2**0, 2*t2**1, 3*t2**2,  -0, -t2**0, -2*t2**1, -3*t2**2], #  velocity continuity at mid point
            [0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0,    0, 0,     2*t2**0, 6*t2**1,  -0, -0,     -2*t2**0, -6*t2**1], #  acceleration continuity at mid point
            

            [0, 0, 0, 0,  0, 0, 0, 0,   t3**0, t3**1, t3**2, t3**3,     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  end X position
            [0, 0, 0, 0,  0, 0, 0, 0,   0, 0, 0, 0,                     0, 0, 0, 0,  0, 0, 0, 0,  t3**0, t3**1, t3**2, t3**3], #  end Y position

        ])

        amax=AMAX

        self.b = np.array([
            self.wps[0][0], 0,0,    self.wps[0][1], self.wps[0][1], self.wps[0][2], self.wps[0][2], 0,0,0,0, 
            self.wps[1][0], 0,0,    self.wps[1][1], self.wps[1][1], self.wps[1][2], self.wps[1][2], 0,0,0,0,
        ])

        self.G = np.array([

            # Position endpoint within geofence
            [0, 0, 0, 0,  0, 0, 0, 0,   t3**0, t3**1, t3**2, t3**3,     0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  end X position
            [0, 0, 0, 0,  0, 0, 0, 0,   0, 0, 0, 0,                     0, 0, 0, 0,  0, 0, 0, 0,  t3**0, t3**1, t3**2, t3**3], #  end Y position
            [0, 0, 0, 0,  0, 0, 0, 0,   -t3**0, -t3**1, -t3**2, -t3**3, 0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 0, 0], #  end X position
            [0, 0, 0, 0,  0, 0, 0, 0,   0, 0, 0, 0,                     0, 0, 0, 0,  0, 0, 0, 0,  -t3**0, -t3**1, -t3**2, -t3**3,], #  end Y position

            # --- wp 1 acceleration box ---
            # [0, 0, 0, 0,    0, 0, 2*t1**0, 6*t1**1,   0, 0, 0, 0,       0, 0, 0, 0,  0, 0, 0, 0,                0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0,    0, 0, -2*t1**0, -6*t1**1, 0, 0, 0, 0,       0, 0, 0, 0,  0, 0, 0, 0,                0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0,    0, -0, 0, 0,              0, 0, 0, 0,       0, 0, 0, 0,  0, 0, 2*t1**0, 6*t1**1,    0, 0, 0, 0,], #  acceleration continuity at mid point
            # [0, 0, 0, 0,    0, -0, 0, 0,              0, 0, 0, 0,       0, 0, 0, 0,  0, 0, -2*t1**0, -6*t1**1,  0, 0, 0, 0,], #  acceleration continuity at mid point

            # wp 0 acceleration box
            # [0, 0, 2*t0**0, 6*t0**1,    0, 0, 0, 0,   0, 0, 0, 0,     0, 0, 0, 0,                0, 0, 0, 0,   0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, -2*t0**0, -6*t0**1,  0, 0, 0, 0,   0, 0, 0, 0,     0, 0, 0, 0,                0, 0, 0, 0,   0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0,                0, 0, 0, 0,   0, 0, 0, 0,     0, 0, 2*t0**0, 6*t0**1,    0, 0, 0, 0,   0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0,                0, 0, 0, 0,   0, 0, 0, 0,     0, 0, -2*t0**0, -6*t0**1,  0, 0, 0, 0,   0, 0, 0, 0], #  acceleration continuity at mid point
            
            # wp 2 acceleration box
            # [-0, -0,  0, 0,   0, 0, 0, 0,   0, 0, 2*t2**0, 6*t2**1,    0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0], #  acceleration continuity at mid point
            # [ -0, -0,  0, 0,  0, 0, 0, 0,   0, 0, -2*t2**0, -6*t2**1,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0,      0, 0, 0, 0,   0, -0, 0, 0,               0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 2*t2**0, 6*t2**1,], #  acceleration continuity at mid point
            # [0, 0, 0, 0,      0, 0, 0, 0,   0, -0, 0, 0,               0, 0, 0, 0,  0, 0, 0, 0,   0, 0, -2*t2**0, -6*t2**1,], #  acceleration continuity at mid point
            
            # wp 3 acceleration box
            # [-0, -0,  0, 0,   0, 0, 0, 0,   0, 0, 2*t3**0, 6*t3**1,    0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0], #  acceleration continuity at mid point
            # [ -0, -0,  0, 0,  0, 0, 0, 0,   0, 0, -2*t3**0, -6*t3**1,  0, 0, 0, 0,  0, 0, 0, 0,    0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0,      0, 0, 0, 0,   0, -0, 0, 0,               0, 0, 0, 0,  0, 0, 0, 0,  0, 0, 2*t3**0, 6*t3**1,], #  acceleration continuity at mid point
            # [0, 0, 0, 0,      0, 0, 0, 0,   0, -0, 0, 0,               0, 0, 0, 0,  0, 0, 0, 0,   0, 0, -2*t3**0, -6*t3**1,], #  acceleration continuity at mid point
            

        ])

        self.h = np.array([
            1,2,1,0,
            # amax, amax,amax, amax, 
            # amax, amax, amax, amax, 
            # amax, amax, amax, amax, 
            # amax, amax, amax, amax, 
            #amax, amax,amax, amax
        ])
         
    def setup_objectives(self):
        t1 = self.t[1]
        t2 = self.t[2]
        t3 = self.t[3]

        self.H = scipy.linalg.block_diag(self.get_acc_H(t1), self.get_acc_H(t2), self.get_acc_H(t3), self.get_acc_H(t1),  self.get_acc_H(t2), self.get_acc_H(t3))
        
        self.f = np.zeros(self.H.shape[0])

    def generate(self, coeffs_raw, dt, order=0):
        """works only for two segments, some hardcoding but fast"""
        C = np.array(coeffs_raw).reshape(self.l, self.m, 2*self.n) # coefficients
        self.tvec = np.arange(0,sum(self.t)+dt, dt)

        diff = [fact(j,order) for j in range(2*self.n)]        
        time_powers = np.concatenate([[0]*order, np.arange(2*self.n -order)])
        pvec = diff*(np.tile(self.tvec[:, np.newaxis], 2*self.n) ** time_powers) # (d/l x order)

        c_mask = (self.tvec>=self.t[1]).astype(int) + (self.tvec>=(self.t[1] + self.t[2])).astype(int)
        print(c_mask)
        traj = np.sum(C[:,c_mask]*pvec, axis=2).T

        return traj
    
    def evaluate(self, dt, order=0):
        """get a specific plan for position, velocity etc."""
        if not self.sol["x"] and self.log_level>=LogLevel.INFO:
            print("Run the optimization first")
            return
        else:
            soln = self.sol["x"]
        plan = self.generate(soln, dt=dt, order = order)
        return plan

class MinJerk2WP2D(MinAcc2WP2D):
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO, a_t = None) -> None:    
        super().__init__(order, waypoints, time, log_level, a_t=a_t)
    
    def setup_constraints(self):
        t0 = self.t[0] # just 0
        t1 = self.t[1]
        t2 = self.t[2] + t1

        self.A = np.array([
            ## X
            # === endpoint constraints ===
            # position
            [t0**0, t0**1, t0**2, t0**3,  t0**4, t0**5,      0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  start position
            # velocity
            [0, t0**0, 2*t0**1, 3*t0**2, 4*t0**3, 5*t0**4,   0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  start velocity
            # acceleration
            [0, 0, 2*t0**0, 6*t0**1, 12*t0**2, 20*t0**3,     0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  start acc
            
            # position
            # [0, 0, 0, 0, 0, 0,     t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  end velocity
            # [0, 0, 0, 0, 0, 0,     0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3, 5*t1**4,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  mid velocity
            # velocity
            [0, 0, 0, 0, 0, 0,     0, t2**0, 2*t2**1, 3*t2**2, 4*t2**3, 5*t2**4,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  end velocity
            # acceleration
            # [0, 0, 0, 0, 0, 0,     0, 0, 2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3,     0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  end acc
            [0, 0, 0, 0, 0, 0,     0, 0, 2*t2**0, 6*t2**1, 12*t2**2, 20*t2**3,     0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  end acc
            
            

            # ===== continuity constraints =====
            [0, 0, 0, 0, 0, 0,      t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5,      0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], # mid position
            [t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5,     0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], # mid position

            # [0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2,   -0, -0,     -0,       0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  jerk continuity at mid point

            [0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3,  5*t1**4,    -0, -t1**0, -2*t1**1, -3*t1**2, -4*t1**3,  -5*t1**4,     0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3,   -0, -0,     -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  jerk continuity at mid point
            [0, 0,     0,       0      , 24*t1**0, 120*t1**1,   -0, -0,     -0,       0,       -24*t1**0, -120*t1**1,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  snap continuity at mid point
            # [0, 0,     0,       0      , 0       , 60*t1**2,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  jerk continuity at mid point

            ## Y
            # === endpoint constraints ===
            # position
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     t0**0, t0**1, t0**2, t0**3,  t0**4, t0**5,      0, 0, 0, 0, 0, 0], #  start position
            # velocity
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, t0**0, 2*t0**1, 3*t0**2, 4*t0**3, 5*t0**4,   0, 0, 0, 0, 0, 0], #  start velocity
            # acceleration
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 2*t0**0, 6*t0**1, 12*t0**2, 20*t0**3,     0, 0, 0, 0, 0, 0], #  start acc
            
            # position
            # [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5], #  end position
            # velocity
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,     0, t2**0, 2*t2**1, 3*t2**2, 4*t2**3,  5*t2**4], #  end velocity
            # acceleration
            # [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,     0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3], #  end acc
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,     0, 0,     2*t2**0, 6*t2**1, 12*t2**2, 20*t2**3], #  end acc
            

            # ===== continuity constraints =====
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0,      t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5], # position
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5,     0, 0, 0, 0, 0, 0], # position

            # [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2,   -0, -0,     -0,       0, 0, 0], #  jerk continuity at mid point


            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3,  5*t1**4,    -0, -t1**0, -2*t1**1, -3*t1**2, -4*t1**3,  -5*t1**4],  #  velocity continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3,   -0, -0,     -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2], #  jerk continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0,     0,       0      , 24*t1**0, 120*t1**1,   -0, -0,     -0,       0      , -24*t1**0, -120*t1**1], #  snap continuity at mid point

        ])

        self.b = np.array([
            self.wps[0][0], 0, 0, 0,0,   self.wps[0][1], self.wps[0][1], 0,0, 0,0,
            self.wps[1][0], 0, 0, 0,0,   self.wps[1][1], self.wps[1][1], 0,0, 0,0,
        ])

        self.G = np.array([

            # Position endpoint within geofence
            [0, 0, 0, 0, 0, 0,    t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5], # end Y position
            [0, 0, 0, 0, 0, 0,    -t2**0, -t2**1, -t2**2, -t2**3,  -t2**4, -t2**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     -t2**0, -t2**1, -t2**2, -t2**3,  -t2**4, -t2**5], # end Y position

            # --- wp 1 acceleration box ---
            [0, 0, 0, 0, 0, 0,    0, 0, 2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3,     0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3,  0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,   0, 0,  2*t1**0,  6*t1**1,  12*t1**2,  20*t1**3], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,   0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3], #  acceleration continuity at mid point

            # wp 0 acceleration box
            [0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3,  0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3,  0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3,  0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3,  0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            
            # wp 2 acceleration box
            [0, 0, 0, 0, 0, 0,     0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,     0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3,], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3,], #  acceleration continuity at mid point
            
            # wp mid 1 acceleration box
            # [0, 0, 0, 0, 0, 0,     0, 0,  2*((t2+t1)/4)**0,  6*((t2+t1)/4)**1,  12*((t2+t1)/4)**2,  20*((t2+t1)/4)**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, -2*((t2+t1)/4)**0, -6*((t2+t1)/4)**1, -12*((t2+t1)/4)**2, -20*((t2+t1)/4)**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0,  2*((t2+t1)/4)**0,  6*((t2+t1)/4)**1,  12*((t2+t1)/4)**2,  20*((t2+t1)/4)**3], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0, -2*((t2+t1)/4)**0, -6*((t2+t1)/4)**1, -12*((t2+t1)/4)**2, -20*((t2+t1)/4)**3], #  acceleration continuity at mid point
            
            # wp mid 2 acceleration box
            # [0, 0, 0, 0, 0, 0,     0, 0,  2*(3*(t2+t1)/4)**0,  6*(3*(t2+t1)/4)**1,  12*(3*(t2+t1)/4)**2,  20*(3*(t2+t1)/4)**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, -2*(3*(t2+t1)/4)**0, -6*(3*(t2+t1)/4)**1, -12*(3*(t2+t1)/4)**2, -20*(3*(t2+t1)/4)**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0,  2*(3*(t2+t1)/4)**0,  6*((3*t2+t1)/4)**1,  12*(3*(t2+t1)/4)**2,  20*(3*(t2+t1)/4)**3], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0, -2*(3*(t2+t1)/4)**0, -6*((3*t2+t1)/4)**1, -12*(3*(t2+t1)/4)**2, -20*(3*(t2+t1)/4)**3], #  acceleration continuity at mid point
            
            # wp mid 3 acceleration box
            # [0, 0, 0, 0, 0, 0,     0, 0,  2*((t2+t1)/2)**0,  6*((t2+t1)/2)**1,  12*((t2+t1)/2)**2,  20*((t2+t1)/2)**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, -2*((t2+t1)/2)**0, -6*((t2+t1)/2)**1, -12*((t2+t1)/2)**2, -20*((t2+t1)/2)**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0,  2*((t2+t1)/2)**0,  6*((t2+t1)/2)**1,  12*((t2+t1)/2)**2,  20*((t2+t1)/2)**3], #  acceleration continuity at mid point
            # [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,                                 0, 0, 0, 0, 0, 0,     0, 0, -2*((t2+t1)/2)**0, -6*((t2+t1)/2)**1, -12*((t2+t1)/2)**2, -20*((t2+t1)/2)**3], #  acceleration continuity at mid point
            
        ])

        self.h = np.array([
            1,2,1,0,
            AMAX, AMAX,AMAX, AMAX, 
            AMAX, AMAX,AMAX, AMAX,
            AMAX, AMAX,AMAX, AMAX, 
            # AMAX, AMAX,AMAX, AMAX,
            # AMAX, AMAX,AMAX, AMAX,
            # AMAX, AMAX,AMAX, AMAX, 
            #amax, amax,amax, amax
        ])
    
    def get_acc_H(self, t):
        return 2*np.array([
            [0, 0, 0,  0,          0,        0       ],
            [0, 0, 0,  0,          0,        0       ],
            [0, 0, 0,  0,          0,        0       ],
            [0, 0, 0,  36*t,       72*t**2,  120*t**3],
            [0, 0, 0,  72*(t**2),  192*t**3, 360*t**4],
            [0, 0, 0,  120*(t**3), 360*t**4, 720*t**5]
        ])
    
    def get_vel_H(self, t):
        return np.array([
            [0, 0,       0,       0,            0,       0],
            [0, 1,       2*t**1,  3*t**2,  4*t**3,  5*t**4],
            [0, 2*t**1,  4*t**2,  6*t**3,  8*t**4, 10*t**5],
            [0, 3*t**2,  6*t**3,  9*t**4, 12*t**5, 15*t**6],
            [0, 4*t**3,  8*t**4, 12*t**5, 16*t**6, 20*t**7],
            [0, 5*t**4, 10*t**5, 15*t**6, 20*t**7, 25*t**8]
        ])
    
    def get_acc(self, t):
        return 2*np.array([
            [0, 0,  0,       0,        0,        0],
            [0, 0,  0*t**0,  0*t**0,   0*t**0,   0*t**0],
            [0, 0,  4*t**0,  12*t**1,  24*t**2,  40*t**3],
            [0, 0,  12*t**1, 36*t**2,  72*t**3,  120*t**4],
            [0, 0,  24*t**2, 72*t**3,  144*t**4, 240*t**5],
            [0, 0,  40*t**3, 120*t**4, 240*t**5, 400*t**6]
        ])
     
    def setup_objectives(self):
        t1 = self.t[1]
        t2 = self.t[2] 
        self.H = scipy.linalg.block_diag(0.01*self.get_acc_H(t1), 1*self.get_acc(t1), 0.01*self.get_acc_H(t1), 1*self.get_acc(t1))

        self.f = np.zeros(self.H.shape[0])
        a_t_x = self.a_t[0]
        a_t_y = self.a_t[1]

        # self.a_t = np.array([a_t_x, a_t_y])
        if self.log_level >= LogLevel.DEBUG:
            print("Target acc. at interception: ", a_t_x, a_t_y)
        self.f = np.array([
            0,0,0,0,0,0,
            0, 0, -4*a_t_x, -12*t1*a_t_x, -24*a_t_x*t1**2, -40*a_t_x*t1**3,
            0,0,0,0,0,0,
            0, 0, -4*a_t_y, -12*a_t_y*t1, -24*a_t_y*t1**2, -40*a_t_y*t1**3,
            ])

class MinSnap2WP2D(MinAcc2WP2D):
    def setup_constraints(self):
        t0 = self.t[0] # just 0
        t1 = self.t[1]
        t2 = self.t[2] + t1

        self.A = np.array([
            ## X
            # === endpoint constraints ===
            # position
            [t0**0, t0**1, t0**2, t0**3,  t0**4, t0**5, t0**6, t0**7,      0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  start position
            [0, t0**0, 2*t0**1, 3*t0**2, 4*t0**3, 5*t0**4, 6*t0**5, 7*t0**6,   0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            [0, 0, 2*t0**0, 6*t0**1, 12*t0**2, 20*t0**3, 30*t0**4, 42*t0**5,     0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  start acc
            [0, 0, 0, 6*t0**0, 24*t0**1, 60*t0**2, 120*t0**3, 210*t0**4,     0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  start jerk
            
            # positionself.A
            # [0, 0, 0, 0, 0, 0,     t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  end velocity
            # [0, 0, 0, 0, 0, 0,     0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3, 5*t1**4,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  mid velocity
            # velocity
            [0, 0, 0, 0, 0, 0, 0, 0,   0, t2**0, 2*t2**1, 3*t2**2, 4*t2**3, 5*t2**4, 6*t2**5, 7*t2**6,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  end velocity
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 2*t2**0, 6*t2**1, 12*t2**2, 20*t2**3, 30*t2**4, 42*t2**5,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  end acc
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0,       6*t2**0, 24*t2**1, 60*t2**2, 120*t2**3, 210*t2**4,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  end acc
            

            # ===== continuity constraints =====
            [0, 0, 0, 0, 0, 0, 0, 0,      t1**0, t1**1, t1**2, t1**3, t1**4, t1**5, t1**6, t1**7,      0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0], # mid position
            [t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5, t1**6, t1**7,    0, 0, 0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0], # mid position

            [0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3,   5*t1**4,  6*t1**5, 7*t1**6,    -0, -t1**0, -2*t1**1, -3*t1**2, -4*t1**3,  -5*t1**4, -6*t1**5, -7*t1**6,            0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3,  30*t1**4, 42*t1**5,   -0, -0,     -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3, -30*t1**4, -42*t1**5,        0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2,  120*t1**3, 210*t1**4,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2, -120*t1**3, -210*t1**4,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  jerk continuity at mid point
            [0, 0,     0,       0,       24*t1**0, 120*t1**1, 360*t1**2, 840*t1**3,   -0, -0,     -0,       0,       -24*t1**0, -120*t1**1, -360*t1**2, -840*t1**3,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  snap continuity at mid point


            ## Y
            # === endpoint constraints ===
            # position
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     t0**0, t0**1, t0**2, t0**3,  t0**4, t0**5, t0**6, t0**7,       0, 0, 0, 0, 0, 0, 0, 0], #  start position
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, t0**0, 2*t0**1, 3*t0**2, 4*t0**3, 5*t0**4, 6*t0**5, 7*t0**6,  0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 2*t0**0, 6*t0**1, 12*t0**2, 20*t0**3, 30*t0**4, 42*t0**5,    0, 0, 0, 0, 0, 0, 0, 0], #  start acc
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 6*t0**0, 24*t0**1, 60*t0**2, 120*t0**3, 210*t0**4,     0, 0, 0, 0, 0, 0, 0, 0], #  start jerk
            
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,     0, t2**0, 2*t2**1, 3*t2**2, 4*t2**3,  5*t2**4, 6*t2**5, 7*t2**6], #  end velocity
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     2*t2**0, 6*t2**1, 12*t2**2, 20*t2**3, 30*t2**4, 42*t2**5], #  end acc
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     0,       6*t2**0, 24*t2**1, 60*t2**2, 120*t2**3, 210*t2**4], #  end acc            

            # ===== continuity constraints =====
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,      t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5, t1**6, t1**7], # position
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5, t1**6, t1**7,     0, 0, 0, 0, 0, 0, 0, 0], # position

            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3,   5*t1**4,  6*t1**5,   7*t1**6,    -0, -t1**0, -2*t1**1, -3*t1**2, -4*t1**3,   -5*t1**4,    -6*t1**5,   -7*t1**6],  #  velocity continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3, 30*t1**4,  42*t1**5,   -0, -0,     -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3,   -30*t1**4,  -42*t1**5], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2, 120*t1**3, 210*t1**4,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2,  -120*t1**3, -210*t1**4], #  jerk continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     0,       0,       24*t1**0, 60*t1**2, 360*t1**2, 840*t1**3,   -0, -0,     -0,       -0,       -24*t1**0, -120*t1**1, -360*t1**2, -840*t1**3], #  jerk continuity at mid point

        ])


        self.b = np.array([
            self.wps[0][0], 0, 0, 0, 0, 0, 0,   self.wps[0][1], self.wps[0][1], 0,0,0,0,
            self.wps[1][0], 0, 0, 0, 0, 0, 0,   self.wps[1][1], self.wps[1][1], 0,0,0,0
        ])


        self.G = np.array([

            # Position endpoint within geofence
            [0, 0, 0, 0, 0, 0, 0, 0,    t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5, t2**6, t2**7,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5, t2**6, t2**7], # end Y position
            [0, 0, 0, 0, 0, 0, 0, 0,    -t2**0, -t2**1, -t2**2, -t2**3,  -t2**4, -t2**5, -t2**6, -t2**7,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     -t2**0, -t2**1, -t2**2, -t2**3,  -t2**4, -t2**5, -t2**6, -t2**7], # end Y position

            # --- wp 1 acceleration box ---
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3, 30*t1**4, 42*t1**5,     0, 0, 0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3, -30*t1**4,  -42*t1**5,  0, 0, 0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0,  2*t1**0,  6*t1**1,  12*t1**2,  20*t1**3, 30*t1**4,  -2*t1**5], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3, -30*t1**4,  -42*t1**5], #  acceleration continuity at mid point

            # wp 0 acceleration box
            [0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3, 30*t0**4,  42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3, -30*t0**4,  -42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3, 30*t0**4,  42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3, -30*t0**4,  -42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            
            # wp 2 acceleration box
            [0, 0, 0, 0, 0, 0, 0, 0,     0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3, 30*t2**4,  42*t2**5,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,     0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3, -30*t2**4,  -42*t2**5,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,                          0, 0, 0, 0, 0, 0, 0, 0,     0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3, 30*t2**4,  42*t2**5], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,                          0, 0, 0, 0, 0, 0, 0, 0,     0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3, -30*t2**4,  -42*t2**5], #  acceleration continuity at mid point
            
        ])

        self.h = np.array([
            1,2,1,0,
            AMAX, AMAX,AMAX, AMAX, 
            AMAX, AMAX,AMAX, AMAX, 
            AMAX, AMAX,AMAX, AMAX, 
        ])
    
    def get_acc_H(self, t):
        return 2*np.array([
            [0, 0, 0, 0,  0,           0,        0,       0],
            [0, 0, 0, 0,  0,           0,        0,       0],
            [0, 0, 0, 0,  0,           0,        0,       0],
            [0, 0, 0, 0,  0,           0,        0,       0],
            [0, 0, 0, 0,  576*t,       1440*t**2, 2880*t**3, 5070*t**4],
            [0, 0, 0, 0,  1440*(t**2), 4800*t**3, 10800*t**4, 20160*t**5],
            [0, 0, 0, 0,  2880*(t**3), 10800*t**4,  25920*t**5, 50400*t**6],
            [0, 0, 0, 0,  5070*(t**4), 20160*t**5, 50400*t**6, 100800*t**7],
        ])
    
    def get_acc(self, t):
        return 2*np.array([
            [0, 0,  0,  0,   0,   0,   0, 0],
            [0, 0,  0,  0,   0,   0,   0, 0],
            [0, 0,  4*t**0,  12*t**1,  24*t**2,  40*t**3 , 60*t**4, 84*t**5],
            [0, 0,  12*t**1, 36*t**2,  72*t**3,  120*t**4, 180*t**5, 252*t**6],
            [0, 0,  24*t**2, 72*t**3,  144*t**4, 240*t**5, 360*t**6, 504*t**7],
            [0, 0,  40*t**3, 120*t**4, 240*t**5, 400*t**6, 600*t**7, 840*t**8],
            [0, 0,  60*t**4, 180*t**5, 360*t**6, 600*t**7, 900*t**8, 1260*t**9],
            [0, 0,  84*t**5, 252*t**6, 504*t**7, 840*t**8, 1260*t**9, 1764*t**10]
        ])

    def setup_objectives(self):
        t1 = self.t[1]
        t2 = self.t[2]

        # self.H = scipy.linalg.block_diag(self.get_acc_H(t1), self.get_acc_H(t2), self.get_acc_H(t3), self.get_acc_H(t1),  self.get_acc_H(t2), self.get_acc_H(t3))
        self.H = scipy.linalg.block_diag(self.get_acc_H(0), self.get_acc(t1),  self.get_acc_H(0),  self.get_acc(t1))
        
        self.f = np.zeros(self.H.shape[0])
        a_t = 10
        
        # self.f = -a_t*np.array([
        #     0,0,0,0,0,0,0,0,
        #     0, 0, 4, 12*t1, 24*t1**2, 40*t1**3, 60*t1**4, 84*t1**5,
        #     0,0,0,0,0,0,0,0,
        #     0,0,0,0,0,0,0,0,
        #     0, 0, 4, 12*t1, 24*t1**2, 40*t1**3, 60*t1**4, 84*t1**5,
        #     0,0,0,0,0,0,0,0,
        #     ])

class MinSnap3WP2D(MinSnap2WP2D):
    def setup_constraints(self):
        t0 = self.t[0] # just 0
        t1 = self.t[1]
        t2 = self.t[2] + t1
        t3 = self.t[3] + t2

        # acc_t = wp

        self.A = np.array([
            ## X
            # === endpoint constraints ===
            # position
            [t0**0, t0**1, t0**2, t0**3,  t0**4, t0**5, t0**6, t0**7,         0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start position
            [0, t0**0, 2*t0**1, 3*t0**2, 4*t0**3, 5*t0**4, 6*t0**5, 7*t0**6,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            [0, 0, 2*t0**0, 6*t0**1, 12*t0**2, 20*t0**3, 30*t0**4, 42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start acc
            [0, 0, 0, 6*t0**0, 24*t0**1, 60*t0**2, 120*t0**3, 210*t0**4,      0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start jerk
            
            # [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3, 30*t1**4, 42*t1**5,    0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start acc


            # positionself.A
            # [0, 0, 0, 0, 0, 0,     t3**0, t3**1, t3**2, t3**3,  t3**4, t3**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  end velocity
            # [0, 0, 0, 0, 0, 0,     0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3, 5*t1**4,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #  mid velocity
            # velocity
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, t3**0, 2*t3**1, 3*t3**2, 4*t3**3, 5*t3**4, 6*t3**5, 7*t3**6,   0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  end velocity
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 2*t3**0, 6*t3**1, 12*t3**2, 20*t3**3, 30*t3**4, 42*t3**5,   0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  end acc
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0,       6*t3**0, 24*t3**1, 60*t3**2, 120*t3**3, 210*t3**4,   0, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  end jerk
            

            # ===== continuity constraints =====
            [0, 0, 0, 0, 0, 0, 0, 0,      t1**0, t1**1, t1**2, t1**3, t1**4, t1**5, t1**6, t1**7,  0, 0, 0, 0, 0, 0, 0, 0,        0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], # mid position
            [t1**0, t1**1, t1**2, t1**3, t1**4, t1**5, t1**6, t1**7,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,        0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], # mid position
            [0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,      t2**0, t2**1, t2**2, t2**3, t2**4, t2**5, t2**6, t2**7,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], # mid position
            [0, 0, 0, 0, 0, 0, 0, 0,      t2**0, t2**1, t2**2, t2**3, t2**4, t2**5, t2**6, t2**7,    0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], # mid position



            [0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3,   5*t1**4,  6*t1**5, 7*t1**6,    -0, -t1**0, -2*t1**1, -3*t1**2, -4*t1**3,  -5*t1**4, -6*t1**5, -7*t1**6,          0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,], #  velocity continuity at mid point
            [0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3,  30*t1**4, 42*t1**5,   -0, -0,     -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3, -30*t1**4, -42*t1**5,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,], #  acceleration continuity at mid point
            [0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2,  120*t1**3, 210*t1**4,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2, -120*t1**3, -210*t1**4,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,], #  jerk continuity at mid point
            [0, 0,     0,       0,       24*t1**0, 120*t1**1, 360*t1**2, 840*t1**3,   -0, -0,     -0,       0,       -24*t1**0, -120*t1**1, -360*t1**2, -840*t1**3,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,], #  snap continuity at mid point

            [0, 0, 0, 0, 0, 0, 0, 0,   0, t2**0, 2*t2**1, 3*t2**2, 4*t2**3,   5*t2**4,  6*t2**5, 7*t2**6,    -0, -t2**0, -2*t2**1, -3*t2**2, -4*t2**3,  -5*t2**4, -6*t2**5, -7*t2**6,            0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0], #  velocity continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0,     2*t2**0, 6*t2**1, 12*t2**2, 20*t2**3,  30*t2**4, 42*t2**5,   -0, -0,     -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3, -30*t2**4, -42*t2**5,        0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0,     0,       6*t2**0, 24*t2**1, 60*t2**2,  120*t2**3, 210*t2**4,   -0, -0,     -0,       -6*t2**0, -24*t2**1, -60*t2**2, -120*t2**3, -210*t2**4,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0], #  jerk continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0,     0,       0,       24*t2**0, 120*t2**1, 360*t2**2, 840*t2**3,   -0, -0,     -0,       0,       -24*t2**0, -120*t2**1, -360*t2**2, -840*t2**3,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0], #  snap continuity at mid point


            ## Y
            # === endpoint constraints ===
            # position
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     t0**0, t0**1, t0**2, t0**3,  t0**4, t0**5, t0**6, t0**7,         0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start position
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, t0**0, 2*t0**1, 3*t0**2, 4*t0**3, 5*t0**4, 6*t0**5, 7*t0**6,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start velocity
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 2*t0**0, 6*t0**1, 12*t0**2, 20*t0**3, 30*t0**4, 42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start acc
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 6*t0**0, 24*t0**1, 60*t0**2, 120*t0**3, 210*t0**4,      0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  start jerk    

            # [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3, 30*t1**4, 42*t1**5,   0, 0, 0, 0, 0, 0, 0, 0], #  start acc


            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, t3**0, 2*t3**1, 3*t3**2, 4*t3**3,  5*t3**4, 6*t3**5, 7*t3**6], #  end velocity
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0,     2*t3**0, 6*t3**1, 12*t3**2, 20*t3**3, 30*t3**4, 42*t3**5], #  end acc
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0,     0,       6*t3**0, 24*t3**1, 60*t3**2, 120*t3**3, 210*t3**4], #  end acc            

            # ===== continuity constraints =====
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,      t1**0, t1**1, t1**2, t1**3, t1**4, t1**5, t1**6, t1**7,   0, 0, 0, 0, 0, 0, 0, 0], # position
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     t1**0, t1**1, t1**2, t1**3,  t1**4, t1**5, t1**6, t1**7,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], # position
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,      t2**0, t2**1, t2**2, t2**3, t2**4, t2**5, t2**6, t2**7], # position
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,      t2**0, t2**1, t2**2, t2**3, t2**4, t2**5, t2**6, t2**7,   0, 0, 0, 0, 0, 0, 0, 0], # position
  
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, t1**0, 2*t1**1, 3*t1**2, 4*t1**3,   5*t1**4,  6*t1**5,   7*t1**6,    -0, -t1**0, -2*t1**1, -3*t1**2, -4*t1**3,   -5*t1**4,    -6*t1**5,   -7*t1**6,  0, 0, 0, 0, 0, 0, 0, 0,],  #  velocity continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3, 30*t1**4,  42*t1**5,   -0, -0,     -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3,   -30*t1**4,  -42*t1**5,  0, 0, 0, 0, 0, 0, 0, 0,], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     0,       6*t1**0, 24*t1**1, 60*t1**2, 120*t1**3, 210*t1**4,   -0, -0,     -0,       -6*t1**0, -24*t1**1, -60*t1**2,  -120*t1**3, -210*t1**4,  0, 0, 0, 0, 0, 0, 0, 0,], #  jerk continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0,     0,       0,       24*t1**0, 60*t1**2, 360*t1**2, 840*t1**3,   -0, -0,     -0,       -0,       -24*t1**0, -120*t1**1, -360*t1**2, -840*t1**3,  0, 0, 0, 0, 0, 0, 0, 0,], #  jerk continuity at mid point
           
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0,    t2**0, 2*t2**1, 3*t2**2, 4*t2**3,   5*t2**4,  6*t2**5,   7*t2**6,    -0, -t2**0, -2*t2**1, -3*t2**2, -4*t2**3,   -5*t2**4,    -6*t2**5,   -7*t2**6],  #  velocity continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0,    0,     2*t2**0, 6*t2**1, 12*t2**2, 20*t2**3, 30*t2**4,  42*t2**5,   -0, -0,     -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3,   -30*t2**4,  -42*t2**5], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0,    0,     0,       6*t2**0, 24*t2**1, 60*t2**2, 120*t2**3, 210*t2**4,   -0, -0,     -0,       -6*t2**0, -24*t2**1, -60*t2**2,  -120*t2**3, -210*t2**4], #  jerk continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0, 0,    0,     0,       0,       24*t2**0, 60*t2**2, 360*t2**2, 840*t2**3,   -0, -0,     -0,       -0,       -24*t2**0, -120*t2**1, -360*t2**2, -840*t2**3], #  jerk continuity at mid point

        ])

        a_t = 15*(self.wps[:,2] - self.wps[:,1])/np.linalg.norm((self.wps[:,2]-self.wps[:,1]))
        # a_t = [10,10]
        # print(a_t, np.linalg.norm(a_t))
        
        self.b = np.array([
            self.wps[0][0], 0, 0, 0, 0, 0, 0,   self.wps[0][1], self.wps[0][1], self.wps[0][2], self.wps[0][2], 0,0,0,0, 0,0,0,0,
            self.wps[1][0], 0, 0, 0, 0, 0, 0,   self.wps[1][1], self.wps[1][1], self.wps[1][2], self.wps[1][2], 0,0,0,0, 0,0,0,0
        ])


        self.G = np.array([

            # Position endpoint within geofence
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   t3**0, t3**1, t3**2, t3**3,  t3**4, t3**5, t3**6, t3**7,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,         0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    t3**0, t3**1, t3**2, t3**3,  t3**4, t3**5, t3**6, t3**7], # end Y position
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   -t3**0, -t3**1, -t3**2, -t3**3,  -t3**4, -t3**5, -t3**6, -t3**7,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,        0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    -t3**0, -t3**1, -t3**2, -t3**3,  -t3**4, -t3**5, -t3**6, -t3**7], # end Y position

            # --- wp 1 acceleration box ---
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 2*t1**0, 6*t1**1, 12*t1**2, 20*t1**3, 30*t1**4, 42*t1**5,        0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3, -30*t1**4,  -42*t1**5, 0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0,  2*t1**0,  6*t1**1,  12*t1**2,  20*t1**3, 30*t1**4,  -2*t1**5,     0, 0, 0, 0, 0, 0, 0, 0, ], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,   0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3, -30*t1**4,  -42*t1**5,   0, 0, 0, 0, 0, 0, 0, 0, ], #  acceleration continuity at mid point

            # wp 0 acceleration box
            [0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3, 30*t0**4,  42*t0**5,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3, -30*t0**4,  -42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,      0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3, 30*t0**4,  42*t0**5,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,    0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3, -30*t0**4,  -42*t0**5,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3, 30*t2**4,  42*t2**5,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3, -30*t2**4,  -42*t2**5,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3, 30*t2**4,  42*t2**5], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3, -30*t2**4,  -42*t2**5], #  acceleration continuity at mid point
            
            
            # wp 2 acceleration box
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0,  2*t3**0,  6*t3**1,  12*t3**2,  20*t3**3, 30*t3**4,  42*t3**5,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, -2*t3**0, -6*t3**1, -12*t3**2, -20*t3**3, -30*t3**4,  -42*t3**5,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0,  2*t3**0,  6*t3**1,  12*t3**2,  20*t3**3, 30*t3**4,  42*t3**5], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0, 0,   0, 0, -2*t3**0, -6*t3**1, -12*t3**2, -20*t3**3, -30*t3**4,  -42*t3**5], #  acceleration continuity at mid point
            
        ])

        self.h = np.array([
            1,    2,    1,    0,
            AMAX, AMAX, AMAX, AMAX, 
            AMAX, AMAX, AMAX, AMAX, 
            AMAX, AMAX, AMAX, AMAX, 
            AMAX, AMAX, AMAX, AMAX, 
        ])
    
    def setup_objectives(self):
        t1 = self.t[1]
        t2 = self.t[2]
        t3 = self.t[3]

        # self.H = scipy.linalg.block_diag(self.get_acc_H(t1), self.get_acc_H(t2), self.get_acc_H(t3), self.get_acc_H(t1),  self.get_acc_H(t2), self.get_acc_H(t3))
        self.H = scipy.linalg.block_diag(self.get_acc_H(0), self.get_acc(t1), self.get_acc_H(0), self.get_acc_H(0),  self.get_acc(t1), self.get_acc_H(0))
        
        self.f = np.zeros(self.H.shape[0])
        a_t = 10
        
        # self.f = -a_t*np.array([
        #     0,0,0,0,0,0,0,0,
        #     0, 0, 4, 12*t1, 24*t1**2, 40*t1**3, 60*t1**4, 84*t1**5,
        #     0,0,0,0,0,0,0,0,
        #     0,0,0,0,0,0,0,0,
        #     0, 0, 4, 12*t1, 24*t1**2, 40*t1**3, 60*t1**4, 84*t1**5,
        #     0,0,0,0,0,0,0,0,
        #     ])

class DynamicTraj():
    def __init__(self,ax, st1, st2, wps, x0) -> None:
        self.ax1 = ax["left"]
        self.ax2 = ax["upper right"]
        self.ax3 = ax["lower right"]
        plt.subplots_adjust(left=0.25, bottom=0.55)

        st1.on_changed(self.update_v_theta)
        st2.on_changed(self.update_v_mag)
        plt.connect('button_press_event', self.on_click)
        
        self.wps =wps
        
        # inits
        self.v_theta = st1.val
        self.v_mag = st2.val
        self.x0 = x0
        self.moth_x = -1
        self.moth_y = 1
        self.cost_txt = None
        self.update()

    def update_v_theta(self, val):
        self.v_theta = val
        self.update()

    def update_v_mag(self, val):
        self.v_mag = val
        self.update()

    def update(self):
        # remove pos traj
        for line in self.ax1.lines:
            line.remove()

        # remove arrows
        for line in self.ax1.collections:
            line.remove()
            
        # acc graph
        for line in self.ax3.lines:
            line.remove()

        # vel graph
        for line in self.ax2.lines:
            line.remove()
            
        if self.cost_txt:
            self.cost_txt.remove()

        # self.wps[1][0] = self.end_x
        # self.wps[1][1] = self.end_y
        
        self.moth_origin = np.array([self.moth_x, self.moth_y])
        self.moth_vel = np.array([self.v_theta, self.v_mag])

        # self.wps[2] = self.wps[1]+(self.wps[1]-self.wps[0])*0.2/np.linalg.norm(self.wps[1]-self.wps[0])
        # self.wps[2] = self.wps[1] + np.array([0,0.1])
        # segment_times = [0,self.t1,self.t2]

        # self.traj_opt.set_waypoints(self.wps)
        # self.traj_opt.set_segment_times([0,self.t1, self.t2])
        # mvajscp = MinAcc3WP2D(order=2, waypoints=self.wps.T, t=[0,self.t1,self.t2], log_level=LogLevel.NONE)

        # nonlinear optimization
        start = time.perf_counter()
        nl_optimizer = GDOptTraj(wps, self.x0, LogLevel.NONE, self.moth_origin, self.moth_vel)
        x_opt, cost = nl_optimizer.optimize()
        self.t1 = x_opt[0]
        # mvajscp = MinJerk2WP2D(order=3, waypoints=self.wps.T, time=segment_times, log_level=LogLevel.NONE, theta_t=self.theta_opt)
        mvajscp = nl_optimizer.traj_opt
        # cost = mvajscp.optimize() + 0*sum(segment_times)
        dt = 0.05
        pos = mvajscp.evaluate(order = 0, dt=dt)
        vel = mvajscp.evaluate(order = 1, dt=dt)
        acc = mvajscp.evaluate(order = 2, dt=dt)

        # print(acc)
        # print(np.linalg.norm(acc, axis=1))
        ax2.set_xlim([0, sum(x_opt[:2])])

        ax3.set_xlim([0, sum(x_opt[:2])])

        # plot side graphs   
        
        # acc. trajectory 
        self.ax3.plot(mvajscp.tvec, acc[:,0],'r-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, acc[:,1],'b-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, np.linalg.norm(acc, axis=1),'k-',linewidth=3)

        # target acceleration at interception
        self.ax3.plot(x_opt[0], mvajscp.a_t[0],'rx',markersize=7)        
        self.ax3.plot(x_opt[0], mvajscp.a_t[1],'bx',markersize=7)

        self.ax3.plot([x_opt[0],x_opt[0]], [-25, 25], "k", linestyle='--')

        # velocity trajectory
        self.ax2.plot(mvajscp.tvec, vel[:,0],'r-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, vel[:,1],'b-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, np.linalg.norm(vel, axis=1),'k-',linewidth=3)
        self.ax2.plot([x_opt[0], x_opt[0]],[-5, 5], "k", linestyle='--')
        
        # plot main trajectory
        max_acc = np.max(np.abs(acc), axis=0)
        mvajscp.plot(pos, ax=self.ax1)
        self.ax1.plot(pos[-1][0], pos[-1][1], "ro", markersize=6)
        # print(mvajscp.wps.shape)
        self.ax1.plot([self.moth_origin[0], mvajscp.wps[0][1]], [self.moth_origin[1], mvajscp.wps[1][1]], "k", linestyle="--")
        self.ax1.plot(self.moth_origin[0], self.moth_origin[1] ,"kx", markersize=4)
        
        mvajscp.plot_vec(pos, vel,ax=self.ax1 ,color='gray')
        mvajscp.plot_vec(pos, acc,ax=self.ax1 ,color='k')
        
        self.cost_txt = plt.gcf().text(0.5, 0.95, f"cost: {cost} | x_opt: {x_opt} \n amax: {max_acc} | a_t: {mvajscp.a_t}", horizontalalignment='center', verticalalignment='center', transform=self.ax1.transAxes)    
        plt.pause(0.00001) # need dis boi here

    def on_click(self, event):
        if event.button is MouseButton.LEFT and event.inaxes==self.ax1:
            self.moth_x = event.xdata
            self.moth_y = event.ydata
            self.update()

class GDOptTraj():
    def __init__(self, wps, x0, log_level, moth_origin, moth_vel) -> None:
        self.wps = wps
        self.x0 = x0
        self.moth_origin = moth_origin
        self.moth_vel = moth_vel
        # calc 'moving forward' bounds for theta opt
        vec_incoming = self.wps[1] - self.wps[0]
        theta_incoming = np.arctan2(vec_incoming[1], vec_incoming[0])
        

        self.bounds = ((0.1, 1.8), (0.2, 1), (0,1), (0,1))
        self.cons = {
            # "type":'eq', 'fun':lambda x:  x[0] + x[1] - sum(x0),
            "type":'ineq', 'fun':lambda x:  x[0] - x[1],
        }
        self.cons = None
        self.log_level = log_level

    def get_gradient(self, x):
        h = 0.02
        
        # this formulation is used when we want to constrain total time
        # in the original paper this takes a more general form with equal splitting of times among segments
        # but we only have two segments to it's chill
        # x_inc = np.array([
        #     [x[0] + h, x[1] - h],
        #     [x[0] - h, x[1] + h],
        # ])
        x_inc = np.array([
            [x[0] + h, x[1]],
            [x[0], x[1] + h],
        ])
        # x_inc = np.clip(x_inc, 0.1, None)
        x_cost = self.objective(x) # precompute
        grad = [(self.objective(x_inc[i]) - x_cost)/h for i in range(len(x))]
        if self.log_level>=LogLevel.DEBUG:
            print("grad",grad)
        return grad

    def objective(self, x):
        start = time.perf_counter()

        moth_vel = self.moth_vel[1]*np.array([np.cos(self.moth_vel[0]), np.sin(self.moth_vel[0])])
        self.wps[1] = self.moth_origin + moth_vel*x[0]
        self.traj_opt = MinJerk2WP2D(order=3, waypoints=self.wps.T, time=[0,x[0],x[1]], log_level=self.log_level, a_t = x[2:])
        
        kinematic_cost = self.traj_opt.optimize()
        t1_cost = x[0]**2
        t2_cost = x[1]**2
        acc_cost = (x[-1])**2
        # print(kinematic_cost, t1_cost, t2_cost,acc_cost)
        cost = 0.1*kinematic_cost + 2*t1_cost + 0.1*t2_cost - 0.1*acc_cost
        
        if self.log_level>=LogLevel.DEBUG:
            print("Optimization time for 1 iteration: ", time.perf_counter() - start)
            print("Total cost: ", cost)
        return cost

    def optimize(self):     
        res = scipy.optimize.minimize(self.objective, self.x0, method='SLSQP', jac='2-point', options={'gtol': 1e-6, 'disp': True},constraints=self.cons, bounds = self.bounds)        

        if self.log_level >=LogLevel.INFO:
            print("Optimal val:", res.x)
        return res.x, res.fun
        
if __name__=="__main__":
    wps = np.array([
        [0, -0.7, 1],
        [0, 1.5, 0.5],
        # [5., 2.],        
        ]).T
    
    
    wps_sorted, _, idx = np.unique(wps, axis=0, return_index=True, return_inverse=True)
    if len(idx)!=len(wps):
        wps = wps_sorted[idx[:-1],:]
    else:
        wps = wps_sorted[idx,:]


    # starting value
    time_opt = [0.5, 0.5]
    x_opt = time_opt + [0.5, 0.5]

    
    fig, axes, = plt.subplot_mosaic([['left','upper right'],
                               ['left','lower right']],
                              figsize=(10.5, 5.5), layout="tight")
    ax1 = axes["left"]
    ax2 = axes["upper right"]
    ax3 = axes["lower right"]
    
    ax1.set_xlim([-1.2, 1.2])
    ax1.set_ylim([-0.3, 2.2])

    ax2.set_xlim([0, sum(x_opt[:2])])
    ax2.set_ylim([-5, 5])
    ax2.grid(True)
    ax3.set_xlim([0, sum(x_opt[:2])])
    ax3.set_ylim([-25, 25])
    ax3.grid(True)

    rect = patches.Rectangle((-1, 0), 2, 2, linewidth=1, edgecolor='r', facecolor='none', linestyle='dashed')
    # Add the patch to the Axes
    ax1.add_patch(rect)

    # Interactive plot
    axfreq = plt.axes([0.1, 0.1, 0.35, 0.03])
    axamp = plt.axes([0.1, 0.13, 0.35, 0.03])  

    # these need to be outside of a function to work (for some crazy reason
    st1 = Slider(axfreq, 'dir', 0, 6.28, valinit=0, valstep=0.1)
    st2 = Slider(axamp, 'mag', 0, 2, valinit=0.5, valstep=0.1)

    dtraj = DynamicTraj(axes, st1, st2, wps, x0 = x_opt)
    print("SDFGd")
    plt.show()
    
    # traj_opt = MinVelAccJerkSnapCrackPop(order = 3, waypoints = wps.T, time= [0,1,2], theta_t=x_opt[-1])
    # print(traj_opt.optimize())