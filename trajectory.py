import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from cvxopt import solvers, matrix
from itertools import product

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
class TrajectoryManager():
    def __init__(self, order, waypoints, time=1) -> None:
        
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
        
    def plot(self, points)	:
        _, dims = points.shape
        if dims == 3: # 3D
            self.ax = plt.axes(projection='3d')
            # ax.set_xlim(-1.5,1.5)
            # ax.set_ylim(-1.5,1.5)
            # ax.set_zlim(-1.5,1.5)
            self.ax.plot(self.wps[0],self.wps[1],self.wps[2],'b--')
            self.ax.plot(points[:,0],points[:,1],points[:,2], 'r-', linewidth=6)
        elif dims == 2:
            plt.plot(self.wps[0],self.wps[1],'k--', linewidth=2)
            plt.plot(points[:,0],points[:,1], 'b-', linewidth=3)
            plt.plot(self.wps[0],self.wps[1],'ko')
        elif dims == 1:
            """plot against time"""
            pass
        #plt.show()

    def plot_vec(self, pos, vec, color='k'):
        for i in range(len(pos)): 
            if i%2==0:
                # plt.arrow(pos[i,0], pos[i,1], vec[i,0], vec[i,1], color=color)
                self.ax.quiver(pos[i,0], pos[i,1], pos[i,2], vec[i,0], vec[i,1], vec[i,2], color=color, arrow_length_ratio=0.1)

class MinVelAccJerkSnapCrackPop(TrajectoryManager): # cute name
    def __init__(self, order, waypoints, time = 1) -> None:
        super().__init__(order, waypoints, time)

        # setup constraints
        print("[Optimizer] Setting up constraints")
        self.setup_constraints()

        # setup objectives
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



if __name__=="__main__":
    wps = np.array([
        [0.,1.,4.,7.],
        [0.,4.,2.,7.],
        [0.,5.,5.,7.],
    ]).T

    # wps = np.array([[ 0.6       , -0.6,         0.5       ],
    #                 [ 0.636843  , -0.33789508,  0.62039724],
    #                 [-0.17777323,  0.55622033,  0.75059858],
    #                 [-0.6       ,  0.6,         0.5      ]])
    wps_sorted, _, idx = np.unique(wps, axis=0, return_index=True, return_inverse=True)
    if len(idx)!=len(wps):
        wps = wps_sorted[idx[:-1],:]
    else:
        wps = wps_sorted[idx,:]
    mvajscp = MinVelAccJerkSnapCrackPop(order=2, waypoints=wps.T, time=8)
    # mvajscp = MinVelAccJerkSnapCrackPopCorridor(order=2, waypoints=wps.T, time=5)
    pos = mvajscp.optimize(plan_order = 0, num_pts=20)
    # Uncomment the following lines to plot further derivatives
    vel = mvajscp.optimize(plan_order = 1, num_pts=20)
    #acc = mvajscp.optimize(plan_order = 2, num_pts=20)
    # jerk = mvajscp.optimize(plan_order = 3, num_pts=200)
    # snap = mvajscp.optimize(plan_order = 4, num_pts=200)
    
    #plt.figure()
    #mvajscp.plot_vec(pos, vel, color='gray')
    #mvajscp.plot(pos)
    
    plt.figure()
    mvajscp.plot(pos)
    mvajscp.plot_vec(pos, vel, color='gray')    
    plt.show()
    
