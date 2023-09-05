import numpy as np
import time
import matplotlib.pyplot as plt

def fact(j, i):
    """factorial of j upto i. Used for derivatives"""
    if i==0:
        return 1
    return j*fact(j-1, i-1)

def pow(n, k):
    if k<0:
        return 1
    return n**k


class TrajectoryGenerator():
    def __init__(self, n, wps, time, d = 100) -> None:
        self.wps = np.array(wps) # time constrained waypoints
    
        self.l  = self.wps.shape[0] # number of dimensions
        print("Number of dimensions: ", self.l)
        self.m = self.wps.shape[1] # number of waypoints
        print("Number of waypoints: ", self.m)
        self.n = n # order of control/number of derivatives
        print("Order of control: ", self.n)

        # time_wts = 
        if type(time) is int:
            self.t = np.cumsum(self.get_path_wts()*time) # cumulative times for waypoints 
            self.t = np.insert(self.t, 0,0)
        elif type(time) is list:
            self.t = time
        print(self.t)
        self.constraints = np.zeros((self.l, self.m, self.n)) # a consise tensor with all n-1 derivative

        # doing this stuff here to increase performance. This stuff doesn't depend on constraints
        
        pts_per_poly = d//(self.m-1)
        
        self.num_pts = pts_per_poly*(self.m-1) 
        # the next line discretizes the trajectory based on the nth order polynomial
        self.T = np.tile(np.linspace(self.t[:-1],self.t[1:], pts_per_poly).T.reshape(self.m-1, pts_per_poly,1), 2*self.n) ** np.arange(2*self.n) # (l x d/l x n)
        self.T_cost = self.T_cost = self.T[:,:,:self.n] # n degree polynomial to calcualate cost 
        self.M_inv = np.linalg.inv(self.M) # inverse of the polynomial tensor. coz we need coefficients given boundary constraints

    def get_path_wts(self):
        """returns distance based weights of the path"""
        wps = self.wps
        diffs = np.linalg.norm(wps[:,1:] - wps[:,:-1], axis=0)
        return diffs/sum(diffs)

    def to_constraints(self, X):
        """
        X: the decision variable vector (a linear vector representation for optimization)
        
        constraints: fully formulated boundary conditions at all points
        axis 0: dimension (x,y,z)
        axis 1: number of points (p1,p2,p3)
        axis 2: number of derivatives (p',p'',p''')

        ####
        example (2,4,4) a 2D minimum snap trajectory for 4 points
        X = [x1',x1'',x1''', x2',x2'',x2''', y1',y1'',y1''', y2',y2'',y2'''] // length = l(m-2)(n-1)

        constraints =
            [[[x0. 0. 0. 0.] // at rest
              [x1. x1',x1'',x1''']
              [x2. x2',x2'',x2''']
              [x3. 0. 0. 0.]] // at rest
  
             [[y0. 0. 0. 0.] // at rest
              [y1. y1',y1'',y1''']
              [y2. y2',y2'',y2''']
              [y3. 0. 0. 0.]] // at rest


        """

        wps = self.wps
        assert len(X) == self.l*(self.m-2)*(self.n-1), "Insufficient or extra number of decision variables"

        self.constraints[:,[0,-1],0] = wps[:,[0,-1]]    # first point is at rest, all n-1 derivatives are zero
        for i in range(self.l): # for all dimensions x,y,z...
            for j in range(self.m - 2): # for all points except first and last
                idx = (i*(self.m-2) + j)*(self.n-1) # this conversion is needed becuase the decision variable is a 1D list, but constraints is 3D
                self.constraints[i,j+1,:] = [wps[i][j+1]] + X[idx:idx + self.n-1]  # updates the n-1 derivatives for all points except first and last
    @property
    def A(self):
        """The boundary condition tensor"""

        # the next line creates the end conditions for all possible derivatives, dimensions and waypoints. (vector on the left side from slides)
        _A = np.array([np.insert(self.constraints[i][1:,:], range(self.n), self.constraints[i][:-1,:], axis=1) for i in range(self.l)])

        return _A.reshape(self.l, self.m-1, 2*self.n, 1)  # reshaped to make it cleaner

    @property
    def M(self):
        """Returns the polynomial tensor"""
        _M = np.empty((len(self.t)-1, 2*self.n, 2*self.n)) 
        for i in range(self.n):
            for j in range(2*self.n):
                _M[:,2*i, j] = fact(j,i) * pow(self.t[:-1], j-i) # time derivatives for start point
                _M[:,2*i+1, j] = fact(j,i) * pow(self.t[1:], j-i) # time derivatives for end point
        return _M
    
    def generate(self):
        C = self.M_inv @ self.A  # (l x n x m-1) The coefficients for each dimension and each polynomial
        self.update_cost(C)
        return (self.T@C).reshape(self.l, self.num_pts).T # (d x l)  

    def update_cost(self, C):
        """Calculate the actual cost over all time. 

        cost = \sum_0^T f^n(x(t)^2) + f^n(y(t)^2) + f^n(z(t)^2)

        here f^n is the nth derivative of the polynomial x(t)

        =====
        for example for minimum snap (n=4)
        x(t) = c_0 + c_1*t + ... c_(2n-1)*t^(2n-1)

        f^4(x(t)) = c_4 + c_5*t + c_6*t^2 + c_7*t^3
                  = [c_4, c_5, c_6, c_7] * [1, t, t^2, t^3]
        Note that we don't use the actual derivative coefficients and merge them into
        the main poly coefficients as it won't matter while minimizing.
        """
        self.cost = np.sum((self.T_cost@C[:,:,self.n:])**2) # equiv

    def plot(self, plan):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(self.wps[0],self.wps[1],self.wps[2],'b-')
        # for p_i in plan:
        ax.plot(plan[:,0],plan[:,1],plan[:,2], 'r-')
        plt.show()

# time bound waypoints
# (x,y,t)   2D


waypoints = np.array([
    # [0,2], # all xs
    [0,1, 1.5], # all xs
    [0,2, 3], # all ys
    [0,2, 3], # all zs
])
v = waypoints[:,1] - waypoints[:,0]
v_cap = v/np.linalg.norm(v)
v_f = v_cap*2
p_f = waypoints[:,1] + v_cap*2
waypoints[:, -1] =p_f
print(v, v_f, p_f)
# l: number of dimensions
# m: number of waypoints
# n: number of derivatives/order

# decision vector
# (n-1)(m-2)l variables without time optimization 

# v = waypoints[:,-1]- waypoints[:, -2]

X = [
    # x1_d, x1_dd, x1_ddd, x2_d, x2_dd ... m-1,
    # y1_d, y1_dd, y1_ddd, y2_d, y2_dd ... m-1,
    # z1_d, z1_dd, z1_ddd, z2_d, z2_dd ... m-1,
    # 0, 0 
    v_f[0],1,0,
    v_f[1],1,0,
    v_f[2],1,0,
] # the optimization vector. can indlude times as well.
# during optimization, velocity and acceleration constraints will be imposed on the magnitudes


if __name__ == "__main__":
    order = 4 # min snap
    time_vec = [0,1,2]
    total_time = time_vec[-1]
    print("sdgsdfgsd")
    ts = 0.05
    tgen = TrajectoryGenerator(n = order, wps = waypoints, time = total_time, d = int(total_time/ts))

    tgen.to_constraints(X)
    start = time.perf_counter()

    traj = tgen.generate() # numpy array of dxl points of the trajectory
    print(traj)
    print(time.perf_counter()-start)

    tgen.plot(traj)
    # fig, ax = plt.figure()
    # plt.plot(tgen.T, traj[:,0])
    # plt.show()



















"""
## arguments to pogram
- model // str: cf2x
- map_id // int: 0,1,2,3
- global_planner // str: 'rrt', 'rrt_star'
- min_snap // bool
- local_planner // str: 'mpc', 'bug', 'orca'

env = Env(model = drone)
env.load_map(map_id) // map has start, goal, 
plan = env.plan(global_planner = global_planner, min_snap = min_snap) // list of x,yz coordinates. run will take care of rest
env.run(plan, local_planner =)

env.plan
    if method is rrt
        wps = RRT(map) # map includes start, goal and obstacles
    elif method is rrt
        wps = RRT_star(map) # map includes start, goal and obstacles

    if min_snap:
        plan = MinSnap(order=4, waypoints = wps).optimize()
    else:
        plan = self.discretise_wps(wps)
    return plan

"""
