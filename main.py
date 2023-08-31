from acados_template import  AcadosSim, AcadosSimSolver
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from models import PATSX
from controllers import MPCController
import scipy
from math import pi


# setup agent and controller

agent = PATSX()
controller = MPCController(agent)

# SIMULATION

sim = AcadosSim()
# set model_name
sim.model = controller.model

# set simulation time
sim.solver_options.T = agent.dt
# set options
sim.solver_options.integrator_type = 'ERK'
sim.solver_options.num_stages = 4
sim.solver_options.num_steps = 3
sim.solver_options.newton_iter = 3 # for implicit integrator
sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"

acados_integrator = AcadosSimSolver(sim)
acados_integrator.set("u", controller.u0)
acados_integrator.set("x", agent.x0)
sim_x = np.zeros((controller.N+1, controller.nx))

# animation
fig = plt.figure()
ax = plt.axes(projection='3d', xlim=(-3.3, 3), ylim=(-3.3, 3), zlim=(-3.3, 3))
ax.view_init(elev=35, azim=-135)
pos, = ax.plot(0,0,0, 'bo', markersize=5)
target_plot, = ax.plot(0,0,0, 'rx', markersize=5)
x_pred_plots = [ax.plot(0,0,0, 'k.', markersize=4)[0] for _ in range(controller.N)]
motor_plots = [ax.plot(0,0,0, 'ro', markersize=2)[0] for _ in range(4)]



def ani_update(i):
    # get new state from observer. Ground truth here
    sim_x = acados_integrator.get("x")    

    acados_integrator.set("x", sim_x)

    # get a new setpoint
    setpoint = np.array([3.0,-3.2,-3.5, 0,0,0, 0, 0, agent.hov_T])

    # calculate optimal control problem
    action = controller.get_action(state_c=sim_x, state_d=setpoint)

    # action = np.array([10, 0, -0.01])
    print(sim_x)
    # update dynamics in simulation/real life
    acados_integrator.set("u", action)
    acados_integrator.solve()

    # animation updates
    pos.set_data_3d(sim_x[0], sim_x[1], sim_x[2])
    target_plot.set_data_3d(setpoint[0], setpoint[1], setpoint[2])

    R = scipy.spatial.transform.Rotation.from_euler('zyx', [sim_x[3], sim_x[4], sim_x[5]])

    # motor_pts_t = R.apply(agent.motor_pos) + np.array([sim_x[0], sim_x[1], sim_x[2]])

    # for i in range(4):
    #     motor_plots[i].set_data_3d(motor_pts_t[i,0], motor_pts_t[i,1], motor_pts_t[i,2])

    for i in range(controller.N):
        x_pred = controller.solver.get(i, "x")
        x_pred_plots[i].set_data_3d(x_pred[0], x_pred[1], x_pred[2])

    return pos, *motor_plots, *x_pred_plots, target_plot


anim = animation.FuncAnimation(fig, ani_update, frames=200, interval=35, blit=True)

plt.show()