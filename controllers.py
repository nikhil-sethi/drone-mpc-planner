from acados_template import AcadosOcpSolver, AcadosOcp, AcadosModel
import casadi as ca
from casadi import *
import numpy as np
from math import pi, cos
from typing import Any
from params import Dynamics, geofence
from models import DynamicsModel, ConstantAccGravity

class Controller:
    def __call__(self, state, **kwds: Any) -> Any:
        raise NotImplementedError

class Constant(Controller):
    def __init__(self, action) -> None:
        self.action = action

    def __call__(self, state, **kwds: Any) -> Any:
        return self.action

class ConstantThetaMag2D(Constant):
    def __call__(self, state, **kwds: Any) -> Any:
        return self.action[1]*np.array([np.cos(self.action[0]), np.sin(self.action[0])])

class Random(Controller):
    def __init__(self, mean = [0,0], variance = [1,1]) -> None:
        self.mu = mean
        self.var = variance 
        
    def __call__(self, state, **kwds: Any) -> Any:
        return np.random.normal(loc=self.mu, scale=[self.var[0]/2, self.var[0]/2],size=len(self.mu)) # divide by two because we want 95% of the velocities to be under v_max

def create_model(model):
    name = "particle_ode"
    # Create the CasADi symbols for the function definition

    # states
    x = ca.MX.sym('x', model.N_STATES)

    # controls
    u = ca.MX.sym('u', model.N_ACTIONS)

    # dynamics model
    xdot = ca.MX.sym('xdot', model.N_STATES)

    # explicit expression. the actual derivative xdot = f(x,u)
    # f_expl = agent.xdot_linear(x,u)
    f_expl = ca.vertcat(*model(x,u))

    model = AcadosModel()
    model.x = x
    model.xdot = xdot

    # this expression is set to 0 and solved. basically just saying that the explicit expression is just = xdot. extra steps
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.u = u
    model.name = name

    # constraints
    con_expr = ca.MX.sym('con_expr', 1)

    # con_expr[0] = u[0]**2 + u[1]**2 + u[2]**2
    # con_expr[1] = x[0]
    # con_expr[0] = ca.sqrt(ca.sumsqr(u))
    # model.con_h_expr = con_expr
    # model.con_h_expr_e = con_expr

    # h = [u[0]**2 + u[1]**2 + u[2]**2]
    # model.set('constr_expr_h', h);
    return model

class MPC(Controller):
    def __init__(self, model:DynamicsModel) -> None:
        self.ocp = AcadosOcp()
        self.model = create_model(model)
        self.ocp.model = self.model

        nx = self.ocp.model.x.size()[0]
        nu = self.ocp.model.u.size()[0]
        self.nx = nx
        self.nu = nu

        ny = nx + nu
        ny_e = nx

        self.N = 10  # number of optimization predictions
          
        Tf = self.N*Dynamics.dt # number of seconds to 'look-ahead' =  time for each step in seconds * number of steps
        Q_mat = 1*np.diag([550, 550, 550, 40, 40, 40])
        R_mat = 0.0001*np.diag([1, 1, 1])

        # self.x0 = agent.x0
        self.u0 = np.array([0,Dynamics.G,0]) # hover

        # set simulation time
        self.ocp.dims.N = self.N
        self.ocp.dims.nx = nx
        self.ocp.dims.ny = ny
        self.ocp.dims.ny_e = ny_e
        # set options
        self.ocp.solver_options.integrator_type = 'ERK' # explicit Runge kutta integration
        self.ocp.solver_options.num_stages = 4 # number of RK4 integration steps
        self.ocp.solver_options.num_steps = 3
        self.ocp.solver_options.newton_iter = 3 # for implicit integrator
        self.ocp.solver_options.collocation_type = "GAUSS_RADAU_IIA"


        # Use the following equations if you're using a custom cost function and not the non linear least sqares matrix that I use
        # path cost
        # self.ocp.model.cost_expr_ext_cost = (model.x - setpoint).T @ Q_mat @ (model.x - setpoint) + (model.u-u_setpoint).T @ R_mat @ (model.u-u_setpoint)
        # terminal cost
        # self.ocp.model.cost_expr_ext_cost_e = (model.x - setpoint).T@ (100*Q_mat) @ (model.x - setpoint)

        # initial setpoint
        self.ocp.cost.yref   = np.array([1, 0.2, 0.5, 0, 0, 0, 0, 0, 0])

        self.ocp.cost.yref_e = np.array([1, 0.2, 0.5, 0, 0, 0])
#

        # self.ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)
        self.ocp.cost.W = np.block([[Q_mat,               np.zeros((nx, nu))],
                                    [np.zeros((nu, nx)), R_mat               ]])

        # MPC terminal cost. Higher values of the weight imply that a lot of focus is given to arriving at the reference state accurately.
        # Depending on the sampling time and the expected distances between the reference and tht drone's position this might need to be tuned
        # a value of zero means that the optimization of N steps happens just in the
        # a very high value is like planning backwards from the final reference setpoint back to the drone's position. Like more of global planning than local. but this clearly might have disadvantages because the plan might not be executed even if it is planned well in theory.
        # I've found 1 to be a good tradeoff between planning in the dark and with some direction.
        self.ocp.cost.W_e = 0*Q_mat


        # The mapping matrix for states.
        # sometimes a reference setpoint can be a function of the state + control, not exactly the same
        # this matrix converts the state to a representation which we give our reference in
        Vx = np.zeros((ny, nx))
        Vx[:nx, :nx] = np.eye(nx)
        self.ocp.cost.Vx = Vx

        # Mapping matrix for control inputs
        Vu = np.zeros((ny, nu))
        # Vu[-3:, -3:] = 1.0
        self.ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        self.ocp.cost.Vx_e = Vx_e

        # Constraints on the RPMs is what helps keep the drone corrected
        # this is a limitation and probably better tuning could allow more room

        phi_max = theta_max = pi/2
        T_max = Dynamics.G*1.2

        # self.ocp.constraints.lbu = np.array([-10, -10, -0])
        # self.ocp.constraints.ubu = np.array([+10, +10, +T_max])

        # self.ocp.dims.np = 1
        self.ocp.constraints.constr_type = 'BGH'
        # self.ocp.model.con_h_expr = ca.vertcat(*[self.ocp.model.u[0]**2 + self.ocp.model.u[1]**2 + self.ocp.model.u[2]**2])
        self.ocp.model.con_h_expr = self.ocp.model.u[0]**2 + self.ocp.model.u[1]**2 + self.ocp.model.u[2]**2
        self.ocp.constraints.lh = np.array([0])
        self.ocp.constraints.uh = np.array([5**2])

        nsh = 1
        self.ocp.constraints.lsh = np.zeros(nsh)             # Lower bounds on slacks corresponding to soft lower bounds for nonlinear constraints
        self.ocp.constraints.ush = np.zeros(nsh)             # Lower bounds on slacks corresponding to soft upper bounds for nonlinear constraints
        self.ocp.constraints.idxsh = np.array(range(nsh))    # Jsh
        ns = 1
        self.ocp.cost.zl = 0 * np.ones((ns,)) # gradient wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.ocp.cost.Zl = 0 * np.ones((ns,)) # diagonal of Hessian wrt lower slack at intermediate shooting nodes (1 to N-1)
        self.ocp.cost.zu = 0 * np.ones((ns,))
        self.ocp.cost.Zu = 0 * np.ones((ns,))

        # state constraints
        max_speed = 3
        self.gf = geofence
        self.ocp.constraints.lbx = np.array([self.gf[0][0], self.gf[1][0], self.gf[2][0], -max_speed, -max_speed, -max_speed])
        self.ocp.constraints.ubx = np.array([self.gf[0][1], self.gf[1][1], self.gf[2][1], +max_speed, +max_speed, +max_speed])
        self.ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5])

        u_max = 15

        self.ocp.constraints.lbu = np.array([-20, -0, -20])
        self.ocp.constraints.ubu = np.array([+20, +18, +20])

        self.ocp.constraints.idxbu = np.array([0, 1, 2])

        # set options
        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
        # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
        # self.ocp.solver_options.print_level = 1
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP
        self.ocp.solver_options.qp_solver_cond_N = self.N
        self.ocp.solver_options.print_level = 0 # SQP_RTI, SQP
        # self.ocp.constraints.x0 = self.x0

        self.ocp.solver_options.tf = Tf

        self.solver = AcadosOcpSolver(self.ocp, json_file = 'acados_json', verbose=False)

        self.preds = np.zeros((self.N, self.nx))

    def __call__(self, state_c, traj, i=0):

        # set current state
        self.solver.set(0, "lbx", state_c)
        self.solver.set(0, "ubx", state_c)

        # v_t = state_d[:-6] - state_c[:-3]
        # v_t_cap = v_t/np.linalg.norm(v_t)
        # self.solver.set(1, "u", v_t_cap*10)

        split = self.N-1

        # set current setpoint

        cid = np.argmin(np.linalg.norm(state_c[:3] - traj[:, :3], axis=1))

        idf = min(cid, i)
        for j in range(self.N):

            idx = min(j + idf, len(traj)-1)
            # print(idf, idx)
            yref = traj[idx]
            # yref = np.array([0.85, -0.5,-1.04486, 0,0,0, 0,0,0])

            self.solver.set(j, "yref", yref)

        # for j in range(self.N-split, self.N):
        #     self.solver.set(j, "yref", traj[1])
        idx_n = min(idf + self.N, len(traj)-1)
        yref_n = traj[idx_n][:-3]
        # yref_n = np.array([0.85, -0.5,-1.04486, 0,0,0])

        # print(state_c)
        # print(np.linalg.norm(state_c[3:6]))

        # for j in range(self.N):
        #     yref = np.array([traj[0][j+i][0], traj[0][j+i][1], traj[0][j+i][2], traj[1][j+i][0], traj[1][j+i][1], traj[1][j+i][2], traj[2][j+i][0], traj[2][j+i][1], traj[2][j+i][2]])
        #     self.solver.set(j, "yref", yref)

        # # for j in range(self.N-split, self.N):
        # #     self.solver.set(j, "yref", traj[1])
        # yref_n = np.array([traj[0][-1][0], traj[0][-1][1], traj[0][-1][2], traj[1][-1][0], traj[1][-1][1], traj[1][-1][2]])

        self.solver.set(self.N, "yref", yref_n)

        status = self.solver.solve()
        # print(state_c[3:], traj[min(idf, len(traj)-1), 3:6])

        # the perfect MPC predicted state. not sure which one to use.
        self.x = self.solver.get(1, "x")

        # store predictions
        self.preds = np.zeros((self.N, self.nx))
        for i in range(self.N):
            self.preds[i] = self.solver.get(i, "x")

        self.u = self.solver.get(0, "u")
        # if np.linalg.norm(self.u)>15:
        #     self.u = self.u/np.linalg.norm(self.u)*15
        return self.u

if __name__=="__main__":
    import os
    from agents import PATSX

    # directory change magic so that there aren't a billion cgen code folders
    print("========= Rebulding OCP problem... ======== ")
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    agent = PATSX()
    controller = MPCController(agent=agent)
    print("========== Done ============")
