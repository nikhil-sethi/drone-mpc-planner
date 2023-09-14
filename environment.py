from acados_template import  AcadosSim, AcadosSimSolver
import time
import numpy as np
import scipy
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trajectory import MinVelAccJerkSnapCrackPop

class Env():
    def __init__(self, agent, controller, target) -> None:
        self.agent = agent
        self.controller = controller
        self.moth = target

        self._simulator = AcadosSimSolver(controller.ocp, json_file = "acados_json")
        self.sim_x = agent.x0
        self.u0 = controller.u0

        # self.target = np.array([moth_pos[0], moth_pos[1], moth_pos[2], target_vel[0], target_vel[1], target_vel[2], 0, 0, self.agent.hov_T])

        # RENDERING
        fig = plt.figure(figsize=(10,10))
        ax = plt.axes(projection='3d', xlim=(-1, 1), ylim=(-3.5, 0), zlim=(-1.5, 0))
        ax.view_init(elev=25, azim=90)
        self.agent_plt, = ax.plot(0,0,0, 'bo', markersize=5)
        self.traj_plt = [ax.plot(0,0,0, 'rx', markersize=5)[0] for _ in range(2)]
        self.x_pred_plts = [ax.plot(0,0,0, 'k.', markersize=4)[0] for _ in range(controller.N)]
        self.motor_plts = [ax.plot(0,0,0, 'ro', markersize=2)[0] for _ in range(4)]
        
        gf = (3,3,3)
        x = [gf[0],gf[0],gf[0],gf[0]],   [-gf[0],-gf[0],-gf[0],-gf[0]], [gf[0], -gf[0], -gf[0], gf[0]], [ gf[0], -gf[0],-gf[0], gf[0]]
        y = [gf[1],-gf[1],-gf[1],gf[1]], [gf[1],-gf[1],-gf[1], gf[1]],  [gf[1], gf[1],gf[1], gf[1]],    [-gf[1], -gf[1],-gf[1], -gf[1]]
        z = [gf[2],gf[2],-gf[2],-gf[2]], [gf[2], gf[2],-gf[2],-gf[2]],  [gf[2], gf[2],-gf[2],-gf[2]],   [ gf[2],  gf[2],-gf[2],-gf[2]]
        self.gf = gf
        surfaces = []

        for i in range(len(x)):
            surfaces.append( [list(zip(x[i],y[i],z[i]))] )

        for surface in surfaces:
            ax.add_collection3d(Poly3DCollection(surface, alpha=0.2))

        np.random.seed(1)

        wps = np.array([
        [0., -1., 1],
        [-0.5, -1., -0],        # y axis
        [0., -2., -2.5],     # z axis
        ]).T

        tf = 1.6   # needs to be manually tuned right now within the control input limits
        dt_traj = self.agent.dt
        n = int(tf/dt_traj)+1
        mvajscp = MinVelAccJerkSnapCrackPop(order=2, waypoints=wps.T, time=tf)
        pos = mvajscp.optimize(plan_order = 0, num_pts=n)
        vel = mvajscp.optimize(plan_order = 1, num_pts=n)
        acc = mvajscp.optimize(plan_order = 2, num_pts=n)
        self.traj = np.hstack([pos, vel, acc])
        u_ref = acc

        # for i in range(len(self.traj)):
        #     ax.plot(self.traj[i][0], self.traj[i][1], self.traj[i][2], 'r.', markersize=3)
        
        # mvajscp.plot(pos, ax=ax)
        # mvajscp.plot_vec(pos, vel, ax=ax, color='gray')

        file_path = "/home/nikhil/Nikhil/Masters/Internship/PATS/code/pats/logs/20230914_165615/log_flight1.csv"
        cols = [12, 13, 14, 19, 20,21, 23, 24, 25, 47,48,49, 31, 30, 41,42,43]
        cols.extend(range(72,117))
        data = np.loadtxt(open(file_path, "rt").readlines()[1:-1], delimiter=";", usecols=cols) 

        self.pos_log = data[:,:3]
        self.vel_log = data[:,3:6]
        self.acc_log = data[:,6:9]
        self.control_log = data[:, 9:12]
        # timesteps = data[:, 12]
        self.time_log = data[:, 13]
        self.pos_t_log = data[:, 14:17]
        self.preds_log = data[:, 17:].reshape((len(data), self.controller.N, 3))
        # dt = np.mean(timesteps)


        # self.anim = animation.FuncAnimation(fig, self.update, frames=200, interval=agent.dt*1000, blit=True)
        self.anim = animation.FuncAnimation(fig, self.replay_update, frames=len(data), interval=10, blit=True)

        plt.show()

    def simulate(self, x, u):
        pass


    def render_update(self):
        # ========= Animation updates
        self.agent_plt.set_data_3d(self.sim_x[0], self.sim_x[2], self.sim_x[1])
        
        # for i in range(2):
        #     self.traj_plt[i].set_data_3d(self.traj[i][0], self.traj[i][1], self.traj[i][2])
        # R = scipy.spatial.transform.Rotation.from_euler('zyx', [sim_x[3], sim_x[4], sim_x[5]])

        # motor_pts_t = R.apply(agent.motor_pos) + np.array([sim_x[0], sim_x[1], sim_x[2]])

        # for i in range(4):
        #     motor_plots[i].set_data_3d(motor_pts_t[i,0], motor_pts_t[i,1], motor_pts_t[i,2])

        # for i in range(self.controller.N):
        #     x_pred = self.controller.solver.get(i, "x")
        #     self.x_pred_plts[i].set_data_3d(x_pred[0], x_pred[2], x_pred[1])

        for i in range(self.controller.N):
            self.x_pred_plts[i].set_data_3d(self.x_preds[i][0], self.x_preds[i][2], self.x_preds[i][1])
        
        # return 

    def update(self, i):
        self.moth.update(self.moth.get_action())

        # create new setpoint
        to_moth = -self.sim_x[:3]+self.moth.pos
        dist_to_moth = np.linalg.norm(to_moth)
        target_vel_cap = to_moth/np.linalg.norm(to_moth)
        target_vel = target_vel_cap*3

        moth = np.array([self.moth.pos[0], self.moth.pos[1], self.moth.pos[2], target_vel[0], target_vel[1], target_vel[2], target_vel_cap[0]*10, target_vel_cap[1]*10, target_vel_cap[2]*10+ self.agent.hov_T])
        
        moth_pred_pos = self.moth.pos + target_vel*0.035*5
        moth_pred = np.array([moth_pred_pos[0], moth_pred_pos[1], moth_pred_pos[2],  0, 0, 0,  0, 0, self.agent.hov_T])
        
        # self.traj = [
        #     moth,
        #     moth_pred
        # ]
        

        # ======= calculate optimal control problem
        start = time.time()
        action = self.controller.get_action(state_c=self.sim_x, traj=self.traj, i=i)
        
        print("MPC compute rate: ",1/(time.time()-start))
        self.sim_x = self._simulator.simulate(x = self.sim_x, u = action)

        ani_tuple = self.render_update()
    
        # ====== Sanity checks
        # reset simulation if you catch it or it goes out of bounds
        # if dist_to_moth<0.05: # 5 cm accuracy of catching
        #     # moth_pos = -2 + 2*np.random.rand(3)
        #     self.moth.reset()
        #     self.sim_x = np.zeros(6)
        #     # acados_integrator.set("x", )
        #     # print(acados_integrator.get("x"))
        #     print("caught!")

        # if np.any(np.abs(self.moth.pos) > self.gf):
        #     self.moth.reset()
        #     self.sim_x = np.zeros(6)
        #     # acados_integrator.set("x", np.zeros(6))
        #     print("not caught")

        return self.agent_plt, *self.motor_plts, *self.x_pred_plts, *self.traj_plt


    def replay_update(self, i):
        self.sim_x = self.pos_log[i]
        self.x_preds = self.preds_log[i]
        # action = self.controller.get_action(state_c=self.sim_x, traj=self.traj, i=i)
        print(self.sim_x)
        ani_tuple = self.render_update()
        return self.agent_plt, *self.motor_plts, *self.x_pred_plts, *self.traj_plt

