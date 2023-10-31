from acados_template import  AcadosSim, AcadosSimSolver
import time
import numpy as np
import scipy
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trajectory import MinVelAccJerkSnapCrackPop
import paramiko
import pandas as pd

class Env():
    def __init__(self, agent, controller, target, gf) -> None:
        self.agent = agent
        self.controller = controller
        self.moth = target
        self.gf = gf

        self._simulator = AcadosSimSolver(controller.ocp, json_file = "acados_json")
        self.sim_x = agent.x0
        self.u0 = controller.u0

        # self.target = np.array([moth_pos[0], moth_pos[1], moth_pos[2], target_vel[0], target_vel[1], target_vel[2], 0, 0, self.agent.hov_T])

        # RENDERING
        fig = plt.figure(figsize=(10,10))
        ax = plt.axes(projection='3d', xlim=gf[0], ylim=gf[2], zlim=gf[1])
        ax.view_init(elev=25, azim=90)
        self.agent_plt, = ax.plot(0,0,0, 'bo', markersize=5)
        # self.traj_plt = [ax.plot(0,0,0, 'rx', markersize=5)[0] for _ in range(15)]
        self.traj_plt = ax.plot(np.zeros(26),np.zeros(26),np.zeros(26), 'rx--', markersize=5)[0]
        print(self.traj_plt)
        self.x_pred_plts = [ax.plot(0,0,0, 'k.', markersize=4)[0] for _ in range(controller.N)]
        self.motor_plts = [ax.plot(0,0,0, 'ro', markersize=2)[0] for _ in range(4)]
        
        # gf = (1,2,2)
        # self.gf = [(-1,1), (-1.3, 0), (-2.5, 0)]
        x = [self.gf[0][1],self.gf[0][1], self.gf[0][1], self.gf[0][1]], [self.gf[0][0], self.gf[0][0], self.gf[0][0], self.gf[0][0]], [self.gf[0][1], self.gf[0][0], self.gf[0][0], self.gf[0][1]], [self.gf[0][1], self.gf[0][0],self.gf[0][0], self.gf[0][1]]
        y = [self.gf[2][1],self.gf[2][0], self.gf[2][0], self.gf[2][1]], [self.gf[2][1], self.gf[2][0], self.gf[2][0], self.gf[2][1]], [self.gf[2][1], self.gf[2][1], self.gf[2][1], self.gf[2][1]], [self.gf[2][0], self.gf[2][0],self.gf[2][0], self.gf[2][0]]
        z = [self.gf[1][1],self.gf[1][1], self.gf[1][0], self.gf[1][0]], [self.gf[1][1], self.gf[1][1], self.gf[1][0], self.gf[1][0]], [self.gf[1][1], self.gf[1][1], self.gf[1][0], self.gf[1][0]], [self.gf[1][1], self.gf[1][1],self.gf[1][0], self.gf[1][0]]
        # self.controller.set_geofence(self.gf)
        surfaces = []

        for i in range(len(x)):
            surfaces.append( [list(zip(x[i],y[i],z[i]))] )

        for surface in surfaces:
            ax.add_collection3d(Poly3DCollection(surface, alpha=0.2))

        np.random.seed(1)
        self.x_preds = np.zeros(0)
        self.replay = True

        if self.replay:
            host = "10.13.11.55"
            username = "pats"
            key_path = "/home/nikhil/.ssh/pats_wg_id_ed25519"
            key = paramiko.RSAKey(filename=key_path)
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, 22, username, pkey=key)
            sftp = ssh.open_sftp()

            folder_path = "/home/pats/pats/data/20231030_211821/"
            flight_num = 1

            folder_path = "/home/nikhil/"

            # with sftp.file(folder_path + f"trajectory{flight_num}.csv", 'r') as traj_file:
            #     self.traj = np.loadtxt(traj_file, delimiter=",")

            with open(folder_path + f"log_flight{flight_num}.csv", 'r') as log_file:
                data = pd.read_csv(log_file, delimiter=',')
            print(data)
            self.state_log = data['drone_state_str']
            start = data.index.get_loc(data[data['drone_state_str']=="ns_set_waypoint"].index[0])
            end =data.index.get_loc(data[data['drone_state_str']=="ns_set_waypoint"].index[-1])
            total_time = data['elapsed'][end] -data['elapsed'][start]
            print("Total time: ", total_time)
            self.pos_log = data[['posX_drone', 'posY_drone', 'posZ_drone']].to_numpy()
            self.vel_log = data[['svelX_drone', 'svelY_drone', 'svelZ_drone']].to_numpy()
            self.acc_log = data[['saccX_drone', 'saccY_drone', 'saccZ_drone']].to_numpy()

            print("max_vel: ", np.max(np.linalg.norm(self.vel_log[start:], axis=1)))
            print("max_acc: ", np.max(np.linalg.norm(self.acc_log[start:], axis=1)))
            self.control_log = data[['accX_commanded', 'accY_commanded', 'accZ_commanded']]

            # waypoints
            self.pos_t_log = data[self.state_log=="ns_set_waypoint"][['posX_target', 'posY_target', 'posZ_target']].to_numpy()
            ax.plot(self.pos_t_log[:, 0], self.pos_t_log[:, 2], self.pos_t_log[:, 1], "bx", markersize=6)
            
            # mpc predictions
            preds_log = data.iloc[:-1,data.columns.get_loc('mpc_pred_1_x'):data.columns.get_loc('mpc_pred_10_z')+1].to_numpy()
            self.preds_log = preds_log.reshape((len(data)-1, 10, 3))

            # dynamic trajectory
            traj_log = data.iloc[:-1,data.columns.get_loc('traj_1_x'):data.columns.get_loc('traj_26_z')+1].to_numpy()
            self.traj_log = traj_log.reshape((len(data)-1, 26, 3))

            num_frames = len(data)
            ani_interval = 60
            # self.anim = animation.FuncAnimation(fig, self.replay_update, frames=len(data), interval=20, blit=True)
        
        else:
            wps = np.array([
            [-0.1, -0.7,  0.7],
            [-0.8, -0.3,  -0.7],        # y axis
            [1.02, -1.95, -1.6],     # z axis
            ]).T

            tf = 1.57994   # needs to be manually tuned right now within the control input limits
            dt_traj = self.agent.dt
            n = int(tf/dt_traj)+1
            mvajscp = MinVelAccJerkSnapCrackPop(order=2, waypoints=wps.T, time=tf)
            pos = mvajscp.optimize(plan_order = 0, num_pts=n)
            vel = mvajscp.optimize(plan_order = 1, num_pts=n)
            acc = mvajscp.optimize(plan_order = 2, num_pts=n)
            self.traj = np.hstack([pos, vel, acc])
            # print(self.traj)
            u_ref = acc

            mvajscp.plot(pos, ax=ax)
            mvajscp.plot_vec(pos, vel, ax=ax, color='gray')

            num_frames = 200
            ani_interval = self.agent.dt*1000

        self.anim = animation.FuncAnimation(fig, self.update, frames=num_frames, interval=ani_interval, blit=True)
        
        # dt = np.mean(timesteps)

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
        if self.x_preds.size !=0:
            for i in range(self.controller.N):
                self.x_pred_plts[i].set_data_3d(self.x_preds[i][0], self.x_preds[i][2], self.x_preds[i][1])
        
        # for i in range(15):
        self.traj_plt.set_data_3d(self.x_traj[:,0], self.x_traj[:,2], self.x_traj[:,1])
        
        # return 

    def update(self, i):

        if self.replay:
            self.sim_x = self.pos_log[i]
            self.x_preds = self.preds_log[i]
            self.x_traj = self.traj_log[i]
        else:

            self.moth.update(self.moth.get_action())

            # create new setpoint
            to_moth = -self.sim_x[:3]+self.moth.pos
            dist_to_moth = np.linalg.norm(to_moth)
            target_vel_cap = to_moth/np.linalg.norm(to_moth)
            target_vel = target_vel_cap*3

            moth = np.array([self.moth.pos[0], self.moth.pos[1], self.moth.pos[2], target_vel[0], target_vel[1], target_vel[2], target_vel_cap[0]*10, target_vel_cap[1]*10, target_vel_cap[2]*10+ self.agent.hov_T])
            
            moth_pred_pos = self.moth.pos + target_vel*0.035*5
            moth_pred = np.array([moth_pred_pos[0], moth_pred_pos[1], moth_pred_pos[2],  0, 0, 0,  0, 0, self.agent.hov_T])

            self.x_preds = self.controller.preds #.copy()
            # print(self.x_preds)
            # ======= calculate optimal control problem
            start = time.time()
            action = self.controller.get_action(state_c=self.sim_x, traj=self.traj, i=i)
            
            # print("MPC compute rate: ",1/(time.time()-start))
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

        return self.agent_plt, *self.motor_plts, *self.x_pred_plts, self.traj_plt


    def replay_update(self, i):
        
        # action = self.controller.get_action(state_c=self.sim_x, traj=self.traj, i=i)
        # print(self.sim_x)
        ani_tuple = self.render_update()
        return self.agent_plt, *self.motor_plts, *self.x_pred_plts, *self.traj_plt

