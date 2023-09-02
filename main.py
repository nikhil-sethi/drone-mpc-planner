from acados_template import  AcadosSim, AcadosSimSolver
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from models import PATSX
from controllers import MPCController
import scipy
from math import pi
import time


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

fig = plt.figure(figsize=(10,10))
ax = plt.axes(projection='3d', xlim=(-4, 4), ylim=(-4, 4), zlim=(-4, 4))
ax.view_init(elev=35, azim=-135)
pos, = ax.plot(0,0,0, 'bo', markersize=5)
target_plot, = ax.plot(0,0,0, 'rx', markersize=5)
x_pred_plots = [ax.plot(0,0,0, 'k.', markersize=4)[0] for _ in range(controller.N)]
motor_plots = [ax.plot(0,0,0, 'ro', markersize=2)[0] for _ in range(4)]


gf = (3,3,3)
x = [gf[0],gf[0],gf[0],gf[0]],   [-gf[0],-gf[0],-gf[0],-gf[0]], [gf[0], -gf[0], -gf[0], gf[0]], [ gf[0], -gf[0],-gf[0], gf[0]]
y = [gf[1],-gf[1],-gf[1],gf[1]], [gf[1],-gf[1],-gf[1], gf[1]],  [gf[1], gf[1],gf[1], gf[1]],    [-gf[1], -gf[1],-gf[1], -gf[1]]
z = [gf[2],gf[2],-gf[2],-gf[2]], [gf[2], gf[2],-gf[2],-gf[2]],  [gf[2], gf[2],-gf[2],-gf[2]],   [ gf[2],  gf[2],-gf[2],-gf[2]]
surfaces = []

for i in range(len(x)):
    surfaces.append( [list(zip(x[i],y[i],z[i]))] )

for surface in surfaces:
    ax.add_collection3d(Poly3DCollection(surface, alpha=0.2))



new_pos = np.array([1.5,1,-2.5, 0,0,0, 0, 0, agent.hov_T])

moth_pos = np.array([1.5,1,-2.5])
moth_vel = np.zeros(3)
def ani_update(i):
    global new_pos, moth_pos, moth_vel
    # get new state from observer. Ground truth here
    sim_x = acados_integrator.get("x")    

    acados_integrator.set("x", sim_x)

    # get a new setpoint
    # print(i)
    
    # moth changes it's velocity every 1 second almost
    if i%15 ==0:
        # new_pos = -5 + 10*np.random.rand(3)
        # sampled from a gaussian distribution
        moth_vel = np.random.normal(loc=[0,0,0], scale=[1,1,1],size=(3))
        
    moth_pos += moth_vel*0.035

    setpoint = np.array([moth_pos[0], moth_pos[1], moth_pos[2], 0,0,0, 0, 0, agent.hov_T])

    # if i>=20:
    #     setpoint = np.array([4.5,1,-2.5, 0,0,0, 0, 0, agent.hov_T])
    # else:
    #     setpoint = np.array([3,-1,-2.5, 0,0,0, 2*agent.hov_T, 2*agent.hov_T, 2*agent.hov_T])
    # calculate optimal control problem
    start = time.time()
    action = controller.get_action(state_c=sim_x, state_d=setpoint)
    # print(time.time()-start)
    # action = np.array([10, 0, -0.01])
    # print(action)
    print(np.linalg.norm(action))
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