from acados_template import AcadosOcpSolver, AcadosOcp, AcadosModel
import casadi as ca
from casadi import *
import time
import scipy
import numpy as np
from math import pi, cos

def create_model(agent):
    name = "particle_ode"
    # Create the CasADi symbols for the function definition

    # states
    x = ca.MX.sym('x', agent.N_STATES)

    # controls
    u = ca.MX.sym('u', agent.N_ACTIONS)

    # dynamics model
    xdot = ca.MX.sym('xdot', agent.N_STATES)

    # explicit expression. the actual derivative xdot = f(x,u)
    # f_expl = agent.xdot_linear(x,u)
    f_expl = ca.vertcat(*agent.xdot(x,u))
    
    model = AcadosModel()
    model.x = x
    model.xdot = xdot
    
    # this expression is set to 0 and solved. basically just saying that the explicit expression is just = xdot. extra steps
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.u = u
    model.name = name
    return model

class MPCController():
    def __init__(self, agent=None) -> None:
        ocp = AcadosOcp()
        self.model = create_model(agent)
        ocp.model = self.model

        nx = ocp.model.x.size()[0]
        nu = ocp.model.u.size()[0]
        self.nx = nx
        self.nu = nu
        
        ny = nx + nu
        ny_e = nx

        self.N = 20  # number of optimization predictions
          
        Tf = self.N*agent.dt # number of seconds to 'look-ahead' =  time for each step in seconds * number of steps

        Q_mat = 1*np.diag([10, 10, 15, 1, 1, 3])
        R_mat = 0.000001*np.diag([1, 1, 1])

        self.x0 = agent.x0
        self.u0 = np.array([0,0,0])

        # set simulation time
        ocp.dims.N = self.N
        ocp.dims.nx = nx
        ocp.dims.ny = ny
        ocp.dims.ny_e = ny_e
        # set options
        ocp.solver_options.integrator_type = 'ERK' # explicit Runge kutta integration
        ocp.solver_options.num_stages = 4 # number of RK4 integration steps
        ocp.solver_options.num_steps = 3
        ocp.solver_options.newton_iter = 3 # for implicit integrator
        ocp.solver_options.collocation_type = "GAUSS_RADAU_IIA"


        # Use the following equations if you're using a custom cost function and not the non linear least sqares matrix that I use
        # path cost
        # ocp.model.cost_expr_ext_cost = (model.x - setpoint).T @ Q_mat @ (model.x - setpoint) + (model.u-u_setpoint).T @ R_mat @ (model.u-u_setpoint)
        # terminal cost
        # ocp.model.cost_expr_ext_cost_e = (model.x - setpoint).T@ (100*Q_mat) @ (model.x - setpoint) 

        # initial setpoint
        ocp.cost.yref   = np.array([0, 0.2, 0.5, 0, 0, 0, 0, 0, agent.hov_T])

        ocp.cost.yref_e = np.array([0, 0.2, 0.5, 0, 0, 0])
# 

        ocp.cost.W = scipy.linalg.block_diag(Q_mat, R_mat)

        # MPC terminal cost. Higher values of the weight imply that a lot of focus is given to arriving at the reference state accurately.
        # Depending on the sampling time and the expected distances between the reference and tht drone's position this might need to be tuned
        # a value of zero means that the optimization of N steps happens just in the 
        # a very high value is like planning backwards from the final reference setpoint back to the drone's position. Like more of global planning than local. but this clearly might have disadvantages because the plan might not be executed even if it is planned well in theory.
        # I've found 1 to be a good tradeoff between planning in the dark and with some direction.
        ocp.cost.W_e = 0*Q_mat


        # The mapping matrix for states.
        # sometimes a reference setpoint can be a function of the state + control, not exactly the same
        # this matrix converts the state to a representation which we give our reference in
        Vx = np.zeros((ny, nx))
        Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx = Vx

        # Mapping matrix for control inputs
        Vu = np.zeros((ny, nu))
        Vu[-3:, -3:] = 1.0
        ocp.cost.Vu = Vu

        Vx_e = np.zeros((ny_e, nx))
        Vx_e[:nx, :nx] = np.eye(nx)
        ocp.cost.Vx_e = Vx_e

        # Constraints on the RPMs is what helps keep the drone corrected
        # this is a limitation and probably better tuning could allow more room
        
        phi_max = theta_max = pi/2
        T_max = agent.g0*1.2

        ocp.constraints.lbu = np.array([-T_max, -T_max, -T_max])
        ocp.constraints.ubu = np.array([+T_max, +T_max, +T_max])

        # ocp.constraints.lbu = np.array([-T_max, -T_max, -T_max])
        # ocp.constraints.ubu = np.array([+T_max, +T_max, +T_max])

        ocp.constraints.idxbu = np.array([0, 1, 2])

        # set options
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' # FULL_CONDENSING_QPOASES
        # PARTIAL_CONDENSING_HPIPM, FULL_CONDENSING_QPOASES, FULL_CONDENSING_HPIPM,
        # PARTIAL_CONDENSING_QPDUNES, PARTIAL_CONDENSING_OSQP, FULL_CONDENSING_DAQP
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON' # 'GAUSS_NEWTON', 'EXACT'
        # ocp.solver_options.print_level = 1
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' # SQP_RTI, SQP
        ocp.solver_options.print_level = 0 # SQP_RTI, SQP
        ocp.constraints.x0 = self.x0

        ocp.solver_options.tf = Tf

        self.solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

    def get_action(self, state_c, state_d):

        start = time.time()

        # set current state 
        self.solver.set(0, "lbx", state_c)
        self.solver.set(0, "ubx", state_c)

        # set current setpoint
        for j in range(self.N):
            self.solver.set(j, "yref", state_d)
        self.solver.set(self.N, "yref", state_d[:-self.nu])
        status = self.solver.solve()

        # the perfect MPC predicted state. not sure which one to use.
        self.x = self.solver.get(1, "x")
        self.u = self.solver.get(0, "u")
        
        return self.u

