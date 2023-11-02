from acados_template import  AcadosSim, AcadosSimSolver
import time
import numpy as np
import scipy
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from trajectory import MinVelAccJerkSnapCrackPop, MothObliterator
import paramiko
import pandas as pd
from conf import geofence, Dynamics
import threading
from utils import in_arena

class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class Env():
    def __init__(self, agent, target) -> None:
        self.agent = agent
        self.controller = agent.controller
        self.moth = target
        # self.moth.randomize()

        self.gf = geofence

        self._simulator = AcadosSimSolver(self.controller.ocp, json_file = "acados_json")
        self.sim_x = agent.state
        self.u0 = self.controller.u0

        # self.target = np.array([moth_pos[0], moth_pos[1], moth_pos[2], target_vel[0], target_vel[1], target_vel[2], 0, 0, self.agent.hov_T])

        # RENDERING
        fig = plt.figure(figsize=(10,10))
        ax = plt.axes(projection='3d', xlim=self.gf[0], ylim=self.gf[2], zlim=self.gf[1])
        ax.view_init(elev=25, azim=135)
        self.agent_plt, = ax.plot(0,0,0, 'bo', markersize=6)
        self.moth_plt, = ax.plot(1,0,0, 'kx', markersize=6)

        self.traj_plt = ax.plot(np.zeros(20),np.zeros(20),np.zeros(20), 'r-', markersize=5, alpha=0.5)[0]
        # self.x_pred_plts = [ax.plot(0,0,0, 'k.', markersize=4)[0] for _ in range(agent.controller.N)]
        self.x_pred_plts = ax.plot(np.zeros(self.controller.N), np.zeros(self.controller.N), np.zeros(self.controller.N), marker='', color = 'limegreen', linestyle='-',markersize=5, linewidth=3)[0]

        self.motor_plts = [ax.plot(0,0,0, 'ro', markersize=2)[0] for _ in range(4)]
        # self.timer_txt = plt.figtext(0.5, 0.95, "sfdg", fontsize=14)
        self.timer_txt = ax.text(0.5,0.5,  0.5, "t", bbox={'facecolor':'w', 'alpha':0.5, 'pad':5},
                 ha="center")

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

        # np.random.seed(1)
        self.x_preds = np.zeros(0)
        self.replay = False

        if self.replay:
            host = "pats84"
            username = "pats"
            key_path = "/home/nikhil/.ssh/pats_wg_id_ed25519"
            key = paramiko.RSAKey(filename=key_path)
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, 22, username, pkey=key)
            sftp = ssh.open_sftp()

            folder_path = "/home/pats/pats/data/20231030_104246/"
            flight_num = 1

            # folder_path = "/home/nikhil/Nikhil/Masters/Internship/PATS/code/pats/logs/20230922_180506/"

            with sftp.file(folder_path + f"trajectory{flight_num}.csv", 'r') as traj_file:
                self.traj = np.loadtxt(traj_file, delimiter=",")

            with sftp.file(folder_path + f"log_flight{flight_num}.csv", 'r') as log_file:
                data = pd.read_csv(log_file, delimiter=';')
            
            self.state_log = data['drone_state_str']
            start = data.index.get_loc(data[data['drone_state_str']=="ns_set_waypoint"].index[0])
            end =data.index.get_loc(data[data['drone_state_str']=="ns_set_waypoint"].index[-1])
            total_time = data['elapsed'][end] -data['elapsed'][start]
            print("Total time: ", total_time)
            self.pos_log = data[['posX_drone', 'posY_drone', 'posZ_drone']].to_numpy()
            self.vel_log = data[['svelX_drone', 'svelY_drone', 'svelZ_drone']].to_numpy()
            self.acc_log = data[['saccX_drone', 'saccY_drone', 'saccZ_drone']].to_numpy()

            print("max_vel: ", np.max(np.linalg.norm(self.vel_log[start:end], axis=1)))
            print("max_acc: ", np.max(np.linalg.norm(self.acc_log[start:end], axis=1)))
            self.control_log = data[['accX_commanded', 'accY_commanded', 'accZ_commanded']]

            # waypoints
            self.pos_t_log = data[self.state_log=="ns_set_waypoint"][['posX_target', 'posY_target', 'posZ_target']].to_numpy()
            ax.plot(self.pos_t_log[:, 0], self.pos_t_log[:, 2], self.pos_t_log[:, 1], "bx", markersize=6)
            
            # mpc predictions
            preds_log = data.iloc[:-1,data.columns.get_loc('mpc_pred_1_x'):data.columns.get_loc('mpc_pred_10_z')+1].to_numpy()
            self.preds_log = preds_log.reshape((len(data)-1, 10, 3))

            # dynamic trajectory
            traj_log = data.iloc[:-1,data.columns.get_loc('traj_1_x'):data.columns.get_loc('traj_15_z')+1].to_numpy()
            self.traj_log = traj_log.reshape((len(data)-1, 15, 3))

            num_frames = len(data)
            ani_interval = 60
            # self.anim = animation.FuncAnimation(fig, self.replay_update, frames=len(data), interval=20, blit=True)
        
        else:
            wps = np.array([
            [-0.1, -0.7,  0.7],
            [-0.8, -0.3,  -0.7],     # y axis
            [1.02, -1.95, -1.6],     # z axis
            ]).T

            tf = 1.57994   # needs to be manually tuned right now within the control input limits
            dt_traj = self.agent.dt
            n = int(tf/dt_traj)+1
            # mvajscp = MinVelAccJerkSnapCrackPop(order=2, waypoints=wps.T, time=tf)

            # nl_optimizer = MothObliterator(drone_state = drone_state, moth_state = moth_state, log_level=LogLevel.NONE)
            # x_opt, cost = nl_optimizer.optimize()

            # self.t1 = x_opt[0]
            # mvajscp = nl_optimizer.traj_opt

            # dt = 0.05
            # pos = mvajscp.evaluate(order = 0, dt=dt)

            # vel = mvajscp.evaluate(order = 1, dt=dt)
            # acc = mvajscp.evaluate(order = 2, dt=dt)

            # # pos = mvajscp.optimize(plan_order = 0, num_pts=n)
            # # vel = mvajscp.optimize(plan_order = 1, num_pts=n)
            # # acc = mvajscp.optimize(plan_order = 2, num_pts=n)
            # self.traj = np.hstack([pos, vel, acc])
            # # print(self.traj)
            # u_ref = acc

            # mvajscp.plot(pos, ax=ax)
            # mvajscp.plot_vec(pos, vel, ax=ax, color='gray')

            num_frames = 200
            self.ani_interval = self.agent.dt*1000

        # self.traj_thread = RepeatTimer(0.2, function = self.traj_update) 
        # self.traj_thread = threading.Thread(target = self.traj_update) 
        self.traj_update() # just one run to get started
        # self.traj_thread.start()
        self.anim = animation.FuncAnimation(fig, self.update, frames=num_frames, interval=self.ani_interval, blit=True)

        self.counter = 0
        self.caught = 0
        self.not_caught = 0
        plt.show()

    def simulate(self, x, u):
        pass

    def render_update(self, i):
        # ========= Animation updates
        self.agent_plt.set_data_3d(self.sim_x[0], self.sim_x[2], self.sim_x[1])
        self.moth_plt.set_data_3d(self.moth.state[0], self.moth.state[2], self.moth.state[1])
        # for i in range(2):
        #     self.traj_plt[i].set_data_3d(self.traj[i][0], self.traj[i][1], self.traj[i][2])
        R = scipy.spatial.transform.Rotation.from_euler('zyx', [self.action[0], self.action[1], self.action[2]])

        motor_pts_t = R.apply(agent.motor_pos) + np.array([self.sim_x[0], self.sim_x[1], self.sim_x[2]])

        # for i in range(4):
        #     motor_plots[i].set_data_3d(motor_pts_t[i,0], motor_pts_t[i,1], motor_pts_t[i,2])

        # for i in range(self.controller.N):
        #     x_pred = self.controller.solver.get(i, "x")
        #     self.x_pred_plts[i].set_data_3d(x_pred[0], x_pred[2], x_pred[1])
        # if self.x_preds.size !=0:
        #     for i in range(self.controller.N):
        #         self.x_pred_plts[i].set_data_3d(self.x_preds[i][0], self.x_preds[i][2], self.x_preds[i][1])
        try:
            self.x_pred_plts.set_data_3d(self.x_preds[:,0], self.x_preds[:,2], self.x_preds[:,1])
        except:
            pass
       
        try:
            self.traj_plt.set_data_3d(self.x_traj[:,0], self.x_traj[:,2], self.x_traj[:,1])
            pass
        except:
            pass

        elapsed_time = i*self.agent.dt
        self.timer_txt.set_text(f"{elapsed_time:0.2f} seconds")
        # return 
        

    def traj_update(self):
        # while True:
        die_moth_die = MothObliterator(drone_state = self.sim_x, moth_state = self.moth.init_state)
        x_opt, cost = die_moth_die.optimize()

        self.t1 = x_opt[0]
        mvajscp = die_moth_die.traj_opt

        dt = Dynamics.dt
        pos = mvajscp.evaluate(order = 0, dt=dt)
        vel = mvajscp.evaluate(order = 1, dt=dt)
        acc = mvajscp.evaluate(order = 2, dt=dt)
        self.traj = np.hstack([pos, vel, acc])
        self.x_traj = pos

        # print("max_vel = ", np.max(np.linalg.norm(vel, axis=1)))
        # print("max_acc = ", np.max(np.linalg.norm(acc, axis=1)))
        # print(x_opt)

    def update(self, i):
        
        if self.replay:
            self.sim_x = self.pos_log[i]
            self.x_preds = self.preds_log[i]
            self.x_traj = self.traj_log[i]
        else:
            if i>0:   # because matplotlib takes a while to start and keeps the counter at 0 all the time
                self.moth.update()
                self.counter = self.counter+1
                self.x_preds = self.controller.preds
                
                # ======= calculate optimal control problem
                start = time.time()
                self.action = self.controller(state_c=self.sim_x, traj=self.traj, i=self.counter-1)
                
                # print("MPC compute rate: ",1/(time.time()-start))
                self.sim_x = self._simulator.simulate(x = self.sim_x, u = self.action)
            # self.sim_x = self.agent.init_state.copy()

        ani_tuple = self.render_update(i)
        dist_to_moth = np.linalg.norm(self.sim_x[:3]-self.moth.state[:3])
         
        # ====== Sanity checks
        # reset simulation if you catch it or it goes out of bounds
        if dist_to_moth<0.05: # 5 cm accuracy of catching
            self.moth.randomize()
            self.sim_x = self.agent.reset()    
            self.traj_update()
            self.counter = 0
            self.caught +=1
            print("caught! ", self.caught)

        if not in_arena(self.moth.state):
            self.moth.randomize()
            self.sim_x = self.agent.reset()
            self.traj_update()
            self.counter=0
            self.not_caught += 1
            print("not caught", self.not_caught)

        return self.agent_plt, self.moth_plt, self.x_pred_plts, self.traj_plt, self.timer_txt

    def replay_update(self, i):
        
        # action = self.controller.get_action(state_c=self.sim_x, traj=self.traj, i=i)
        # print(self.sim_x)
        ani_tuple = self.render_update()
        return self.agent_plt, *self.motor_plts, *self.x_pred_plts, *self.traj_plt

