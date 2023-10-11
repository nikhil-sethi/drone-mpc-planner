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
np. set_printoptions(threshold=np. inf, suppress=True, linewidth=np. inf)
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

    def generate(self, coeffs_raw, order = 0, d=100):
        """Takes in optimizer solution and generates a discretized trajectory"""
        C = np.expand_dims((np.array(coeffs_raw).reshape(self.m,2*self.n)),-1) # coefficients

        # d = 40 # discretisation
        pts_per_poly = d//self.m
        tvec = np.linspace(self.t[:-1],self.t[1:], pts_per_poly).T.reshape(self.m, pts_per_poly,1)
        diff = [fact(j,order) for j in range(2*self.n)]
        
        time_powers = np.concatenate([[0]*order, np.arange(2*self.n -order)])
        # print(diff, time_powers)
        pvec = diff*(np.tile(tvec, 2*self.n) ** time_powers) # (d/l x order)

        traj = (pvec@C).flatten()
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
    def __init__(self, order, waypoints, time = 1, log_level=LogLevel.INFO) -> None:
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
        for n in range(self.n):
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

        self.f = np.zeros(2*self.n*self.m)

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
        plan = []
        for l in range(self.l):
            b_ep = [self.wps[l][0]] + [0]*(self.n-1) + [self.wps[l][-1]] + [0]*(self.n-1)
            b_con = []
            for i in range(1, self.m):
                b_con += [self.wps[l][i]]*2 + [0]*self.n 
            b = b_ep + b_con	# continuity RHS
            # print(np.array(self.A))
            self.sol = solvers.qp(P = matrix(self.H), q=matrix(self.f), A=matrix(np.array(self.A), tc='d'), b=matrix(b, tc='d'))

            plan.append(self.generate(self.sol["x"], order = plan_order, d=num_pts))
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
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO) -> None:
        super().__init__(order, waypoints, time, log_level)
        
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

    def get_H_1seg(self, t):
        return 2*np.array([
            [t,        (t**2)/2,  (t**3)/3,  (t**4)/4],
            [(t**2)/2, (t**3)/3,  (t**4)/4,  (t**5)/5],
            [(t**3)/3, (t**4)/4,  (t**5)/5,  (t**6)/6],
            [(t**4)/4, (t**5)/5,  (t**6)/6,  (t**7)/7]
        ])

    def setup_objectives(self):
        t1 = self.t[1]
        # print(t1)
        t2 = self.t[2] 
        self.H = scipy.linalg.block_diag(self.get_H_1seg(t1), self.get_H_1seg(t2), self.get_H_1seg(t1), self.get_H_1seg(t2))


        self.f = np.zeros(self.H.shape[0])

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
        elif self.status == 'unknown':
            self.cost = np.inf
        if self.log_level >= LogLevel.INFO:
            print("[Optimizer] Done.")

        return self.cost

    def generate(self, coeffs_raw, dt, order=0):
        """works only for two segments, some hardcoding but fast"""
        C = np.array(coeffs_raw).reshape(self.l, self.m, 2*self.n) # coefficients
        self.tvec = np.arange(0,sum(self.t)+dt, dt)

        diff = [fact(j,order) for j in range(2*self.n)]        
        time_powers = np.concatenate([[0]*order, np.arange(2*self.n -order)])
        pvec = diff*(np.tile(self.tvec[:, np.newaxis], 2*self.n) ** time_powers) # (d/l x order)

        c_mask = (self.tvec>=self.t[1]).astype(int)

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


class DynamicTraj():
    def __init__(self,ax, st1, st2, wps) -> None:
        self.ax1 = ax["left"]
        self.ax2 = ax["upper right"]
        self.ax3 = ax["lower right"]
        plt.subplots_adjust(left=0.25, bottom=0.55)

        st1.on_changed(self.update_t1)
        st2.on_changed(self.update_t2)
        plt.connect('button_press_event', self.on_click)

        self.wps =wps
        
        # inits
        self.t1 = st1.val
        self.t2 = st2.val
        self.end_x = self.wps[1][0]
        self.end_y = self.wps[1][1]
        self.cost_txt = None

    def update_t1(self, val):
        self.t1 = val
        self.update()

    def update_t2(self, val):
        self.t2 = val
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

        self.wps[1][0] = self.end_x
        self.wps[1][1] = self.end_y

        # self.traj_opt.set_waypoints(self.wps)
        # self.traj_opt.set_segment_times([0,self.t1, self.t2])
        mvajscp = MinAcc2WP2D(order=2, waypoints=self.wps.T, time=[0,self.t1,self.t2], log_level=LogLevel.NONE)
        cost = mvajscp.optimize() + 0*(self.t1+self.t2)
        dt = 0.05
        pos = mvajscp.evaluate(order = 0, dt=dt)
        vel = mvajscp.evaluate(order = 1, dt=dt)
        acc = mvajscp.evaluate(order = 2, dt=dt)


        ax2.set_xlim([0, (self.t1+self.t2)])

        ax3.set_xlim([0, (self.t1+self.t2)])

        # plot side graphs    
        self.ax3.plot(mvajscp.tvec, acc[:,0],'r-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, acc[:,1],'b-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, np.linalg.norm(acc, axis=1),'k-',linewidth=3)

        self.ax2.plot(mvajscp.tvec, vel[:,0],'r-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, vel[:,1],'b-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, np.linalg.norm(vel, axis=1),'k-',linewidth=3)

        # plot main trajectory
        max_acc = np.max(np.abs(acc), axis=0)
        mvajscp.plot(pos, ax=self.ax1)
        self.ax1.plot(pos[-1][0], pos[-1][1], "ro", markersize=6)
        # self.ax1.plot(mvajscp.wps[0],mvajscp.wps[1],'k--', linewidth=2)
        # self.ax1.plot(pos[:,0],pos[:,1], 'b-', linewidth=3)
        # self.ax1.plot(mvajscp.wps[0],mvajscp.wps[1],'ko')
        
        mvajscp.plot_vec(pos, vel,ax=self.ax1 ,color='gray')
        mvajscp.plot_vec(pos, acc,ax=self.ax1 ,color='k')
        
        self.cost_txt = plt.gcf().text(0.5, 0.95, f"cost: {cost}, \n amax: {max_acc}", horizontalalignment='center', verticalalignment='center', transform=self.ax1.transAxes)    
        plt.pause(0.00001) # need dis boi here


    def on_click(self, event):
        if event.button is MouseButton.LEFT and event.inaxes==self.ax1:
            self.end_x = event.xdata
            self.end_y = event.ydata
            self.update()


class TimeOptTraj():
    def __init__(self, wps, x0, log_level) -> None:
        self.wps = wps
        self.x0 = x0[1:]
        self.bounds = ((0.1, None), (0.1, None))
        self.cons = {
            # "type":'eq', 'fun':lambda x:  x[0] + x[1] - sum(x0),
            "type":'ineq', 'fun':lambda x:  x[0] - x[1],
        }
        # self.cons = None
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
        return grad

    def objective(self, x):
        start = time.perf_counter()
        self.traj_opt = MinAcc2WP2D(order=2, waypoints=self.wps.T, time=[0,x[0],x[1]], log_level=self.log_level)
        
        kinematic_cost = self.traj_opt.optimize()
        time_cost = np.sum(x**2)
        cost = 0.0*kinematic_cost + 1.0*time_cost
        # print(kinematic_cost, time_cost)
        # acc = self.traj_opt.evaluate(order = 2, dt=dt)
        # max_acc = np.mean(np.linalg.norm(acc, axis=1), axis=0)
        if self.log_level>=LogLevel.DEBUG:
            print("Optimization time for 1 iteration: ", time.perf_counter() - start)
        # print(f"x: {x}, cost:{cost}")
        return cost

    def optimize(self):
        # x0 = [0.7,0.8]
        # res = scipy.optimize.minimize(self.objective, self.x0, method='BFGS', jac=self.get_gradient,
        #        options={'gtol': 1e-6, 'disp': True})        
        res = scipy.optimize.minimize(self.objective, self.x0, method='SLSQP', jac=self.get_gradient,
               options={'gtol': 1e-6, 'disp': True},constraints=self.cons, bounds = self.bounds)        

        return res.x
        # print("Optimal times:", res.x)





if __name__=="__main__":
    wps = np.array([
        [0, 0.6, 0],
        [0., 0.9, 0],
        # [5., 2.],        
        ]).T

    wps_sorted, _, idx = np.unique(wps, axis=0, return_index=True, return_inverse=True)
    if len(idx)!=len(wps):
        wps = wps_sorted[idx[:-1],:]
    else:
        wps = wps_sorted[idx,:]

    time_opt = [0,0.7, 0.2]
    
    # time optimization
    start = time.perf_counter()
    time_opt = TimeOptTraj(wps, time_opt, log_level=LogLevel.NONE).optimize()
    print("Optimal times: ", time_opt)
    print(time.perf_counter()-start)

    # Plot result
    mvajscp = MinAcc2WP2D(order=2, waypoints=wps.T, time=[0, time_opt[0], time_opt[1]])
    mvajscp.optimize()

    dt = 0.05
    pos = mvajscp.evaluate(order = 0, dt=dt)
    # Uncomment the following lines to plot further derivatives
    vel = mvajscp.evaluate(order = 1, dt=dt)
    acc = mvajscp.evaluate(order = 2, dt=dt)
    # jerk = mvajscp.optimize(plan_order = 3, num_pts=200)
    # snap = mvajscp.optimize(plan_order = 4, num_pts=200)

    # print(pos)
    # print(vel)
    # print(acc)
    
    # fig, axes = plt.subplots(1,2)
    fig, axes, = plt.subplot_mosaic([['left','upper right'],
                               ['left','lower right']],
                              figsize=(10.5, 5.5), layout="tight")
    ax1 = axes["left"]
    ax2 = axes["upper right"]
    ax3 = axes["lower right"]
    
    ax1.set_xlim([-1.2, 1.2])
    ax1.set_ylim([-0.3, 2.2])

    ax2.set_xlim([0, sum(time_opt)])
    ax2.set_ylim([-5, 5])
    ax2.grid(True)
    ax3.set_xlim([0, sum(time_opt)])
    ax3.set_ylim([-25, 25])
    ax3.grid(True)


    rect = patches.Rectangle((-1, 0), 2, 2, linewidth=1, edgecolor='r', facecolor='none', linestyle='dashed')
    # Add the patch to the Axes
    ax1.add_patch(rect)

    ax1.plot(pos[-1][0], pos[-1][1], "ro", markersize=6)
    mvajscp.plot(pos, ax=ax1)
    mvajscp.plot_vec(pos, vel,ax=ax1 ,color='gray')    
    mvajscp.plot_vec(pos, acc,ax=ax1 ,color='black')    

    # Interactive plot
    axfreq = plt.axes([0.1, 0.1, 0.35, 0.03])
    axamp = plt.axes([0.1, 0.13, 0.35, 0.03])  
    # these need to be outside of a function to work (for some crazy reason
    st1 = Slider(axfreq, 't1', 0.1, 1.2, valinit=time_opt[0], valstep=0.01)
    st2 = Slider(axamp, 't2', 0.1, 1.2, valinit=time_opt[1], valstep=0.01)

    dtraj = DynamicTraj(axes, st1, st2, wps)
    
    plt.show()
    
