from typing import Any
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
from utils import fact, pow
from conf import *
# from models import FirstOrder
from agents import Moth
# from controllers import Constant, Random
history = []
np.set_printoptions(precision=4, suppress=True)
np.set_printoptions(threshold=np. inf, suppress=True, linewidth=np. inf)
class TrajectoryManager():
    """ Base class for all trajectory planners. Helps in plotting, sanity checks etc."""
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO) -> None:
        
        self.wps = waypoints
        
        self.n_wps = len(waypoints)
        self.l = n_dims
        self.m = self.n_wps - 1 # num segments
        # self.l  = len(waypoints) # number of dimensions
        
        # self.m = len(waypoints[0])-1 # number of segments
        
        assert self.m >=1, "There must be at least two waypoints for a path"
        self.n = order # order of control/number of derivatives
        assert 0<self.n<=4, "Order of polynomial must be between 0 and 4"
        
        if type(time) is int or type(time) is float:
            self.t = self.get_path_wts()*time # cumulative times for waypoints 
            self.t = np.insert(self.t, 0,0)
            
            # time mutation
            # self.t[1]-=0.1
            # self.t[-2]-=0.2
        elif type(time) is list:
            assert len(time) == self.n_wps, "Number of timestamps must be equal to the number of waypoints."
            self.t = time

        self.t_s = np.cumsum(self.t) # stamped times at waypoints
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
        """Evaluate the polynomial defined by coeffs_raw at discretized time instants.
        
        coeffs_raw: coefficients for all segments (m), all dimensions (l) and the right order(n)
        order: derivative order of the polynoimial
        dt: discretization time step
        """
        C = np.array(coeffs_raw).reshape(self.l, self.m, 2*self.n) # coefficients
        self.tvec = np.arange(0, sum(self.t)+dt, dt)  # all time stamps with discretization
        self.tvec[-1] = sum(self.t) # manually add the last point too, can get lost when discretisation is coarse

        diff = [fact(j,order) for j in range(2*self.n)] # TODO: stuff here can be replaced by poly_t_n functions in derived classes. just for intuitiveness
        time_powers = np.concatenate([[0]*order, np.arange(2*self.n -order)])
        pvec = diff*(np.tile(self.tvec[:, np.newaxis], 2*self.n) ** time_powers) # this is polynomial evaluated at all discretized time steps with the 
        c_mask = np.zeros_like(self.tvec)
        for t in self.t[1:-1]:
            c_mask += (self.tvec>=t).astype(int)

        traj = np.sum(C[:, c_mask.astype(int)]*pvec, axis=2).T # coefficients x time^i  at each time stamp is the polynomial. this is just a vectorised version of that.

        return traj
        
    def plot(self, points, ax=None, line_color = 'blue', wp_color = 'red', alpha = 1, linewidth = 4)	:
        _, dims = points.shape
        if dims == 3: # 3D
            # self.ax = plt.axes(projection='3d')
            # ax.set_xlim(-1.5,1.5)
            # ax.set_ylim(-1.5,1.5)
            # ax.set_zlim(-1.5,1.5)
            
            ax.plot(points[:,0],points[:,2],points[:,1], linestyle = '-', color = line_color , alpha = alpha, linewidth=linewidth)
            ax.plot(self.wps[:, 0],self.wps[:, 2],self.wps[:, 1], marker = 'o', color = wp_color, alpha=alpha , markersize=8, linestyle='')
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
            vec = 2*(vec/np.linalg.norm(vec))
            ax.quiver(pos[::2,0], pos[::2,2], pos[::2,1], vec[::2,0], vec[::2,2], vec[::2,1], color=color)
        else:
            ax.quiver(pos[:,0], pos[:,1], vec[:,0],vec[:,1], color=color, width=0.005)

def acc_ic(acc_ic_norm):
    """Calculate acceleration at interception from optimization variables"""

    # the roll and pitch ranges for this calculation could be limited so that search is carried out only 
    # in a certain 3d solid-angle. For example you could choose an area that only allows interception acclerations that are moving forward.
    # might also make search faster + give better solutions. depends on what you want
    ic_phi = Interception.PHI[0] + (Interception.PHI[1]-Interception.PHI[0])*(acc_ic_norm[1]) 
    ic_theta = Interception.THETA[0] + (Interception.THETA[1] - Interception.THETA[0])*(acc_ic_norm[2]) 
    ic_mag =  Interception.MAG[0] + (Interception.MAG[1] - Interception.MAG[0])*acc_ic_norm[0]
    
    return ic_mag * np.array([np.cos(ic_phi), np.sin(ic_phi)*np.cos(ic_theta), -np.sin(ic_phi)*np.sin(ic_theta)])

class MinVelAccJerkSnapCrackPop(TrajectoryManager): # cute name
    """Main trajectory planner
    Uses quadratic programming from CVXOPT to plan a continuous trajectory between multiple waypoints that can minimize snap/jerk etc.
    """
    def __init__(self, order, waypoints, time = 1, log_level=LogLevel.INFO, acc_ic_norm = None) -> None:
        # calc 'moving forward' bounds for theta opt

        # vec_incoming = waypoints[:, 1] - waypoints[:, 0]
        # theta_incoming = np.arctan2(vec_incoming[1], vec_incoming[0])
        
        # ic_theta = (theta_incoming - np.pi/2) + (np.pi) *(acc_ic[0])
        
        self.acc_ic = acc_ic(acc_ic_norm)
        self.acc_ic[1] -= Dynamics.G
        
        super().__init__(order, waypoints, time, log_level)

        self.wps_pos = self.wps[:, :3]
        self.wps_vel = self.wps[:, 3:]

        # setup constraints
        if self.log_level>=LogLevel.INFO:
            print("[Optimizer] Setting up constraints")
        self.setup_eq_constraints()
        self.setup_ineq_constraints()

        # self.obj_wts = [0.01, 0.001] # weights for trajectory smoothing objective (for each segment)
        # self.acc_ic_wt = 3
        # setup objectives
        if self.log_level >=LogLevel.INFO:
            print("[Optimizer] Setting up optimization objective")
        self.setup_objectives()     

    def setup_eq_constraints(self):
        """Sets up the LHS matrix (A) for the equality constraints for the quadratic program of the form A@x = b"""
        # Endpoint constraints
        A_ep = []

        # first waypoint
        for n in range(self.n):
            A_ep.append(self.poly_t_n_m(self.t_s[0], n, 1))

        # last waypoint 
        # no position constraint
        for n in range(1,self.n): 
            A_ep.append(self.poly_t_n_m(self.t_s[self.m], n, self.m))

        # Continuity constraints
        A_con = []
        for m in range(1,self.m):
            # add 2 consecutive position constraints 
            A_pos_m = self.poly_t_n_m(self.t_s[m], Order.POS, m) 
            A_con.append(A_pos_m)
            
            A_pos_m = self.poly_t_n_m(self.t_s[m], Order.POS, m+1)
            A_con.append(A_pos_m)

            for n in range(1, self.n+2):
                # add self.n-1 continuity constraints
                A_con_m = self.poly_t_n_m(self.t_s[m], n, m) - self.poly_t_n_m(self.t_s[m], n, m+1)
                A_con.append(A_con_m)

        # combine endpoint and continuity constraints
        self.A = A_ep + A_con
        
    def setup_ineq_constraints(self):
        """Sets up the LHS matrix (G) for the inequality constraints for the quadratic program of the form G@x <= h"""
        self.G = np.array([

            # geofence constraints
            
            self.poly_t_n_m(self.t_s[1]/2, Order.POS, 1),  # max position at interception 
            -self.poly_t_n_m(self.t_s[1]/2, Order.POS, 1), # min position at interception
            
            self.poly_t_n_m(self.t_s[1], Order.POS, 1),  # max position at interception 
            -self.poly_t_n_m(self.t_s[1], Order.POS, 1), # min position at interception
            
            # following three constraints constrain the second half of the trajectory within geofence
            self.poly_t_n_m(self.t_s[1] + self.t[2]/2, Order.POS, 2),  # max position at interception + 50% 
            -self.poly_t_n_m(self.t_s[1] + self.t[2]/2, Order.POS, 2), # min position at interception + 50%
            
            self.poly_t_n_m(self.t_s[1] + 3*self.t[2]/4, Order.POS, 2),  # max position at interception + 75%
            -self.poly_t_n_m(self.t_s[1] + 3*self.t[2]/4, Order.POS, 2), # min position at interception + 75%
            
            self.poly_t_n_m(self.t_s[1] + self.t[2]/4, Order.POS, 2),  # max position at interception + 25%
            -self.poly_t_n_m(self.t_s[1] + self.t[2]/4, Order.POS, 2), # min position at interception + 25%

            self.poly_t_n_m(self.t_s[-1], Order.POS, 2),  # max position at end point 
            -self.poly_t_n_m(self.t_s[-1], Order.POS, 2), # min position at end point 
            

            # acceleration bo limits at the tree points
            self.poly_t_n_m(self.t_s[0], Order.ACC, 1),  # max acc at start point
            -self.poly_t_n_m(self.t_s[0], Order.ACC, 1), # min acc at start point
            
            self.poly_t_n_m(self.t_s[1], Order.ACC, 2),  # max acc at mid point
            -self.poly_t_n_m(self.t_s[1], Order.ACC, 2), # min acc at mid point
        
            self.poly_t_n_m(self.t_s[-1], Order.ACC, 2),  # max acc at end point
            -self.poly_t_n_m(self.t_s[-1], Order.ACC, 2), # min acc at end point

            # -self.poly_t_n_m(self.t_s[1], Order.VEL, 2), # min position at end point 
            
            ])

    def poly_t_n(self, t, n):
        """nth polynomial derivative at time t"""
        return [fact(j, n)*pow(t, j-n) for j in range(2*self.n)]

    def poly_t_n_m(self, t, n, m):
        """nth polynomial derivative at time t for the m'th segment"""
        return np.array([0]*2*self.n*(m-1) + self.poly_t_n(t, n) + [0]*2*self.n*(self.m - m))

    def acc_quad_t(self, t):
        """Quadratic cost matrix for squared acceleration at time instant t"""
        diff = [fact(j, 2) for j in range(2*self.n)]
        coeff_prods = np.prod(list(product(diff,repeat=2)),-1).reshape(2*self.n,2*self.n)
        
        time_powers = np.sum(np.meshgrid([0,0] + list(range((2*self.n)-2)), [0,0] + list(range((2*self.n)-2))),0) 
        time_poly = t**time_powers # this result comes because of 2n-1 order integration
        
        acc_t = coeff_prods*time_poly
        return acc_t
    
    def setup_objectives(self):
        self.H = np.zeros((2*self.n*self.m, 2*self.n*self.m))
        for m in range(self.m):
            # note that segment times are used, not stamped times.
            self.H[2*self.n*m:2*self.n*(m+1), 2*self.n*m:2*self.n*(m+1)] = Weights.SmoothSeg[m]*self.nth_int_quad_T(self.t[m+1])
    
        # acceleration at interception. Only done for the first segment (m=0)
        self.H[:2*self.n, :2*self.n] += Weights.MatchAccIC*self.acc_quad_t(self.t[1])

        if self.log_level >= LogLevel.DEBUG:
            print("Target acc. at interception: ", self.acc_ic)
    
        self.H *= 2. # to comply with quadratic form

    def set_acc_t(self, acc_ic_l):
        """Sets the linear part of the objective. In our case it's the matching acceleration part (xdd(t) - a_t)^2 = xdd(t)^2 + a_t^2 - 2.a_t.xdd(t) .  Constant part is ignored"""
        self.f = [0, 0] + [Weights.MatchAccIC * (-2*fact(j,2)*acc_ic_l*(self.t_s[1]**(j-2))) for j in range(2,2*self.n)] # comes from linear term of (x_dd(t)-a_t)**2 polynomial
        self.f.extend([0]*2*self.n)
        
    def nth_int_quad_T(self, T):
        """Quadratic Cost matrix for integration of a squared n degree polynomial over a time horizon T"""
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
        if self.log_level >= LogLevel.INFO:
            print("[Optimizer] Starting optimization...")
        self.x_opt = []
        self.status = 1
        self.cost = 0
        for l in range(self.l):   
            # equality constraints RHS
            b_ep = [self.wps_pos[0, l]] + [self.wps_vel[0, l]] + [0]*(self.n-2)  # first waypoint
            b_ep += [self.wps_vel[self.m, l]] + [0]*(self.n-2) # last waypoint (no position)

            b_con = []
            for i in range(1, self.m):
                b_con += [self.wps_pos[i, l]]*2 + [0]*(self.n+1) # continuity RHS
            b = b_ep + b_con	

            if l == 1: # y axis
                # the box around acceleration in the y axis is shifted downwards. This happens because we assume that we're already compensating for gravity.
                # so the amount of acceleration left in the upwards direction is exactly lesser than that amount.
                # this is ofc. not ideal. Ideally you would just upper cap the norm of the entire thing. 
                amax = Dynamics.AMAX - Dynamics.G 
                amin = Dynamics.AMAX + Dynamics.G # this becomes negative
                vmin = Dynamics.VYMIN
            else:
                amax = Dynamics.AMAX
                amin = Dynamics.AMAX   # this becomes negative
                vmin = abs(Dynamics.VYMIN) # used 

            # inequality constraints RHS // box constraints
            h = [abs(geofence[l][1]), abs(geofence[l][0]), 
                 abs(geofence[l][1]), abs(geofence[l][0]), 
                 abs(geofence[l][1]), abs(geofence[l][0]), 
                 abs(geofence[l][1]), abs(geofence[l][0]), 
                 abs(geofence[l][1]), abs(geofence[l][0]), 
                 abs(geofence[l][1]), abs(geofence[l][0]), 
                 amax, amin,  
                 amax, amin, 
                 amax, amin,
                #  vmin
                 ]
                 

            # acceleration at interception depends on dimension
            self.set_acc_t(self.acc_ic[l])
            options = {"show_progress":self.log_level>=LogLevel.DEBUG}

            self.sol = solvers.qp(P = matrix(self.H), q=matrix(self.f, tc='d'), A=matrix(np.array(self.A), tc='d'), b=matrix(b, tc='d'), G=matrix(self.G, tc='d'), h = matrix(h, tc='d'), options=options)
            self.status *= int(self.sol["status"] == "optimal")
            if self.status == 1:
                self.cost += self.sol["primal objective"]
            else:
                self.cost += 50000
            self.x_opt.extend(self.sol["x"])
        
        if self.status == 1:
            # self.cost = self.sol['primal objective']
            pos = self.generate(self.x_opt, dt=0.1, order=0) # coarse discretization is fine 
            acc = self.generate(self.x_opt, dt=0.05, order=2) # need finer discretization because acc is more senstive
            history.append(pos)
            # penalise ovreall acceleration magnitude // is nonlinear
            if np.max(np.linalg.norm(acc, axis=1))>Dynamics.AMAX:
                self.cost += 100000
        
            # manually set last waypoint

            self.wps[-1,:3] = pos[-1]

        if self.log_level >= LogLevel.INFO:
            print("[Optimizer] Done.")
        return self.cost
    
    def plot_history(self, ax):
        """Helps to visualise how trajectories are changing over time. 
        TODO history is a global var right now, shame on you, change this
        """
        for pos in history:
            self.plot(pos, ax, line_color='gray', alpha=0.3, wp_color='gray', linewidth=2)

    def evaluate(self, dt, order=0):
        """get a specific plan for position, velocity etc."""
        if not self.x_opt and self.log_level>=LogLevel.INFO:
            print("Run the optimization first")
            return
        else:
            soln = self.x_opt
        plan = self.generate(soln, dt=dt, order = order)
        if order==2:
            plan[:,1] +=  Dynamics.G # assume thealways compensating for gravity
        return plan

class MinVelAccJerkSnapCrackPopCorridor(MinVelAccJerkSnapCrackPop):
    def __init__(self, order, waypoints, time=1) -> None:
        # discretize waypoints
        num = 2
        temp = np.linspace(waypoints[:,:-1], waypoints[:,1:], num=num, axis=-1, endpoint=False).reshape(waypoints.shape[0], (waypoints.shape[1]-1)*num)
        waypoints = np.append(temp, waypoints[:,-1][:,None], axis=1)
        
        super().__init__(order, waypoints, time)

class MinAcc2WP2D(MinVelAccJerkSnapCrackPop):
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO, acc_ic=None) -> None:
        super().__init__(order, waypoints, time=time, log_level=log_level, acc_ic=acc_ic)
          
    def setup_eq_constraints(self):
        t0 = self.t[0]
        t1 = self.t[1]
        t2 = self.t[2] + t1
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

        self.b = np.array([
            self.wps[0][0], 0,0,    self.wps[0][1], self.wps[0][1], 0,0,
            self.wps[1][0], 0,0,    self.wps[1][1], self.wps[1][1], 0,0,
        ])
    
    def setup_ineq_constraints(self):
        amax=AMAX
        t0 = self.t[0]
        t1 = self.t[1]
        t2 = self.t[2] +t1
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
        self.sol = solvers.qp(G=matrix(self.G, tc='d'), h=matrix(self.h, tc='d'), P = matrix(self.H, tc='d'), q=matrix(self.f), A=matrix(np.array(self.A), tc='d'), b=matrix(self.b, tc='d'), options=options)

        self.status = self.sol['status']
        if self.status == 'optimal':
            self.cost = self.sol['primal objective']
            pos = self.generate(self.sol['x'], dt=0.1, order=0) # coarse discretization is fine 
            acc = self.generate(self.sol['x'], dt=0.05, order=2) # need finer discretization because acc is more senstive

            # penalize geofence //  try to add this in linear quadratic program 
            if np.any(pos > np.array([1,2])+0.1) or np.any(pos<np.array([-1,0])-0.1):
                self.cost += 100000

            # penalise ovreall acceleration magnitude // is nonlinear
            if np.max(np.linalg.norm(acc, axis=1))>Dynamics.AMAX:
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
    def __init__(self, order, waypoints, time=1, log_level=LogLevel.INFO, acc_ic = None) -> None:    
        super().__init__(order, waypoints, time=time, log_level=log_level, acc_ic=acc_ic)
        
    def setup_eq_constraints(self):
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

    def setup_ineq_constraints(self):
        t0 = self.t[0] # just 0
        t1 = self.t[1]
        t2 = self.t[2] + t1

        self.G = np.array([

            # Position endpoint within geofence
            [0, 0, 0, 0, 0, 0,    t2**0,  t2**1,  t2**2,  t2**3,   t2**4,  t2**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #   end X position
            [0, 0, 0, 0, 0, 0,   -t2**0, -t2**1, -t2**2, -t2**3,  -t2**4, -t2**5,   0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0], #   end X position
            
            # wp 0 acceleration box
            [0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3,  0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3,  0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            
             # --- wp 1 acceleration box ---
            [0, 0, 0, 0, 0, 0,    0, 0,  2*t1**0,  6*t1**1,  12*t1**2,  20*t1**3,     0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3,  0, 0, 0, 0, 0, 0,       0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
           
            # wp 2 acceleration box
            [0, 0, 0, 0, 0, 0,     0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,     0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3,  0, 0, 0, 0, 0, 0,     0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            

            # Y
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    t2**0, t2**1, t2**2, t2**3,  t2**4, t2**5], # end Y position
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,   -t2**0, -t2**1, -t2**2, -t2**3,  -t2**4, -t2**5], # end Y position

            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0,  2*t0**0,  6*t0**1, 12*t0**2,  20*t0**3,  0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,     0, 0, -2*t0**0, -6*t0**1,-12*t0**2, -20*t0**3,  0, 0, 0, 0, 0, 0], #  acceleration continuity at mid point
            
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,   0, 0,  2*t1**0,  6*t1**1,  12*t1**2,  20*t1**3], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0,   0, 0, -2*t1**0, -6*t1**1, -12*t1**2, -20*t1**3], #  acceleration continuity at mid point
            
            [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,    0, 0, 0, 0, 0, 0,   0, 0,  2*t2**0,  6*t2**1,  12*t2**2,  20*t2**3,], #  acceleration continuity at mid point
            [0, 0, 0, 0, 0, 0,     0, 0, 0, 0,0, 0,    0, 0, 0, 0, 0, 0,   0, 0, -2*t2**0, -6*t2**1, -12*t2**2, -20*t2**3,], #  acceleration continuity at mid point
            
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
            1,1,
            AMAX, AMAX,
            AMAX, AMAX, 
            AMAX, AMAX,

            2,0,
            AMAX, AMAX,
            AMAX, AMAX,
            AMAX, AMAX, 
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
            [0, 0,  0,       0,        0,        0       ],
            [0, 0,  0*t**0,  0*t**0,   0*t**0,   0*t**0  ],
            [0, 0,  4*t**0,  12*t**1,  24*t**2,  40*t**3 ],
            [0, 0,  12*t**1, 36*t**2,  72*t**3,  120*t**4],
            [0, 0,  24*t**2, 72*t**3,  144*t**4, 240*t**5],
            [0, 0,  40*t**3, 120*t**4, 240*t**5, 400*t**6]
        ])
     
    def setup_objectives(self):
        t1 = self.t[1]
        t2 = self.t[2] 
        self.H = scipy.linalg.block_diag(self.get_acc_H(t1) + self.get_acc(t1), self.get_acc_H(t2), self.get_acc_H(t1) + self.get_acc(t1), self.get_acc_H(t2))
        self.f = np.zeros(self.H.shape[0])
        a_t_x = self.acc_ic[0]
        a_t_y = self.acc_ic[1]
        
        # self.a_t = np.array([a_t_x, a_t_y])
        if self.log_level >= LogLevel.DEBUG:
            print("Target acc. at interception: ", a_t_x, a_t_y)
        self.f = np.array([
            0, 0, -4*a_t_x, -12*t1*a_t_x, -24*a_t_x*t1**2, -40*a_t_x*t1**3,
            0,0,0,0,0,0,
            0, 0, -4*a_t_y, -12*a_t_y*t1, -24*a_t_y*t1**2, -40*a_t_y*t1**3,
            0,0,0,0,0,0,
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
    def __init__(self, axes, st1, st2,) -> None:
        self.ax1 = axes["left"]
        self.ax2 = axes["upper right"]
        self.ax3 = axes["lower right"]

        plt.subplots_adjust(left=0.25, bottom=0.15)
        self.ax1.set_xlim(geofence[0])
        self.ax1.set_ylim(geofence[2][::-1])
        self.ax1.set_zlim(geofence[1])
        
        self.ax3.set_ylim([-Dynamics.AMAX-5, Dynamics.AMAX+5])
        self.ax2.set_ylim([-Dynamics.VMAX-1, Dynamics.VMAX+1])

        self.ax2.grid(True)
        self.ax3.grid(True)

        self.ax1.set_xlabel("x(Width)")
        self.ax1.set_zlabel("y(Height)")
        self.ax1.set_ylabel("z(Depth)")

        self.ax2.set_ylabel("Velocity")
        self.ax3.set_ylabel("Acceleration")
        self.ax3.set_xlabel("Time")
        
        st1.on_changed(self.update_v_theta)
        st2.on_changed(self.update_v_mag)
        plt.connect('button_press_event', self.on_click)
        
        # self.uav_origin = uav_origin
        
        rect = patches.Rectangle(geofence[:2,0], *(geofence[:2,1] - geofence[:2,0]), linewidth=1, edgecolor='r', facecolor='none', linestyle='dashed')
        # Add the patch to the Axes
        # self.ax1.add_patch(rect)

        # inits
        self.v_theta = st1.val
        self.v_mag = st2.val
        self.moth_x = -1
        self.moth_y = 0.5
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
        
        moth_origin = np.array([self.moth_x, self.moth_y])
        moth_vel = np.array([self.v_theta, self.v_mag])

        # nonlinear optimization
        start = time.perf_counter()
        # make sure that the timestep for moth is lesser than lowest t1 bound. because it's an euler simulation
        drone_state = np.array([0, -1 ,  -1  , 0, 0, 0])
        # moth_state = np.array([0.2, -0.7, -1.2, self.v_mag, 0, 0])
        moth_state = np.array([-1.0, -0.3, -1.2, self.v_mag, 0, 0])
        nl_optimizer = MothObliterator(drone_state = drone_state, moth_state =moth_state, log_level=LogLevel.NONE)
        x_opt, cost, status = nl_optimizer.optimize()

        self.t1 = x_opt[0]
        mvajscp = nl_optimizer.traj_opt

        dt = 0.05
        pos = mvajscp.evaluate(order = 0, dt=dt)

        vel = mvajscp.evaluate(order = 1, dt=dt)
        acc = mvajscp.evaluate(order = 2, dt=dt)

        self.ax2.set_xlim([0, sum(x_opt[:2])])
        self.ax3.set_xlim([0, sum(x_opt[:2])])

        # plot side graphs   
        
        # acc. trajectory 
        self.ax3.plot(mvajscp.tvec, acc[:,0],'r-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, acc[:,1],'g-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, acc[:,2],'b-',linewidth=2)
        self.ax3.plot(mvajscp.tvec, np.linalg.norm(acc, axis=1),'k-',linewidth=3)

        # target acceleration at interception
        self.ax3.plot(x_opt[0], mvajscp.acc_ic[0],'rx',markersize=7)        
        self.ax3.plot(x_opt[0], mvajscp.acc_ic[1]+ Dynamics.G,'gx',markersize=7)
        self.ax3.plot(x_opt[0], mvajscp.acc_ic[2],'bx',markersize=7)

        self.ax3.plot([x_opt[0],x_opt[0]], [-Dynamics.AMAX-5, Dynamics.AMAX+5], "k", linestyle='--')

        # velocity trajectory
        self.ax2.plot(mvajscp.tvec, vel[:,0],'r-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, vel[:,1],'g-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, vel[:,2],'b-',linewidth=2)
        self.ax2.plot(mvajscp.tvec, np.linalg.norm(vel, axis=1),'k-',linewidth=3)
        self.ax2.plot([x_opt[0], x_opt[0]],[-Dynamics.VMAX-1, Dynamics.VMAX+1], "k", linestyle='--')
        
        # plot main trajectory
        max_acc = np.max(np.abs(acc), axis=0)
        mvajscp.plot_history(self.ax1)
        mvajscp.plot(pos, ax=self.ax1)
        
        # self.ax1.plot(pos[-1][0], pos[-1][1], "ro", markersize=6)

        moth_pos_history = np.array(nl_optimizer.moth.history["state"])[:, :3]
        self.ax1.plot(moth_pos_history[:,0], moth_pos_history[:,2], moth_pos_history[:,1], 'kx', linestyle="--")

        mvajscp.plot_vec(pos, vel,ax=self.ax1 ,color='gray')
        mvajscp.plot_vec(pos, acc,ax=self.ax1 ,color='k')
        
        self.cost_txt = plt.gcf().text(0.5, 0.95, f"cost: {cost} | x_opt: {x_opt} \n amax: {max_acc} | a_t: {mvajscp.acc_ic}", horizontalalignment='center', verticalalignment='center', transform=self.ax1.transAxes)    
        plt.pause(0.00001) # need dis boi here

    def on_click(self, event):
        if event.button is MouseButton.LEFT and event.inaxes==self.ax1:
            self.moth_x = event.xdata
            self.moth_y = event.ydata
            # self.update()

class MothObliterator():
    def __init__(self, drone_state, moth_state, log_level=LogLevel.NONE) -> None:
        self.drone_state = drone_state
        self.moth = Moth(init_state = moth_state)

        self.log_level = log_level

        # nonlinear optimization params
        self.x0 = [0.5]*5 
        self.x0[-1] = 0.58  # starting pitch angle (try to be close to +g as possible)
        self.bounds = ((0.1, 1.8), (0.2, 1), (0,1), (0,1), (0,1))

        self.cons = {
            # "type":'eq', 'fun':lambda x:  x[0] + x[1] - sum(x0),
            "type":'ineq', 'fun':lambda x:  acc_ic(x[2:])[1] - Dynamics.G # y component of interception accleration should be atleast gravity
        }
        # self.cons = None
        
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
        ic_point = self.moth.simulate(x[0]) # simulate moth for t1 seconds
        wps = np.array([
            self.drone_state,
            ic_point[:], 
            np.empty_like(self.drone_state[:]) # dummy end point. optimized internally
            ])

        self.traj_opt = MinVelAccJerkSnapCrackPop(order = 3, waypoints = wps, time=[0,x[0],x[1]], acc_ic_norm=x[2:], log_level=LogLevel.NONE)

        kinematic_cost = self.traj_opt.optimize()
        t1_cost = x[0]**2
        t2_cost = x[1]**2
        acc_max_cost = (x[2])**2
        cost = 0.1*kinematic_cost + Weights.MinTime[0]*t1_cost + Weights.MinTime[1]*t2_cost - Weights.MaxAccIC*acc_max_cost
        if self.log_level>=LogLevel.DEBUG:
            print("Optimization time for 1 iteration: ", time.perf_counter() - start)
            print("Total cost: ", cost)
        return cost

    def optimize(self):     
        res = scipy.optimize.minimize(self.objective, self.x0, method='SLSQP', jac='2-point', options={'acc': 1e-6, 'disp': False}, constraints=self.cons, bounds = self.bounds)        
        # res = scipy.optimize.shgo(self.objective, self.bounds, constraints = self.cons,n=64, sampling_method='sobol', options={'f_tol': 1e-3, 'disp': True, 'maxtime':0.2})
        if self.log_level >=LogLevel.INFO:
            print("Optimal val:", res.x)
        return res.x, res.fun, res["status"]
        
if __name__=="__main__":

    n_dims = 3
    n_vars = 5

    fig, axes, = plt.subplot_mosaic([['left','upper right'],
                                     ['left','lower right']],
                                     figsize=(10.5, 5.5), layout="tight",
                                     subplot_kw={})
    
    ss = axes["left"].get_subplotspec()
    axes["left"].remove()
    axes["left"] = fig.add_subplot(ss, projection='3d')
        
    # Interactive plot
    ax_dir = plt.axes([0.1, 0.1, 0.35, 0.03])
    ax_mag = plt.axes([0.1, 0.13, 0.35, 0.03])  

    # these need to be outside of a function to work (for some crazy reason
    st1 = Slider(ax_dir, 'dir', 0, 6.28, valinit=0, valstep=0.1)
    st2 = Slider(ax_mag, 'mag', 0, 2, valinit=0.5, valstep=0.1)

    dtraj = DynamicTraj(axes, st1, st2)
    plt.show()
    