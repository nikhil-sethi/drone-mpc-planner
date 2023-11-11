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
from utils import in_arena, unit_vector, quat_from_v1_v2
import matplotlib.gridspec as gridspec

class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)


class Env():
    def __init__(self, agent, target) -> None:
        self.agent = agent
        self.controller = agent.controller
        self.moth = target

        self.gf = geofence

        self._simulator = AcadosSimSolver(self.controller.ocp, json_file = "acados_json")
        self.sim_x = agent.state
        self.action = self.controller.u0
        
        # ======== Rendering =========
        
        fig = plt.figure(figsize=(13,10))
        gs = gridspec.GridSpec(1, 2,width_ratios=[4,1])
        ax = fig.add_subplot(gs[0], projection = '3d', xlim=self.gf[0], ylim=self.gf[2], zlim=self.gf[1])
        ax_stat = fig.add_subplot(gs[1])
        
        ax.set_title('Simulation')
        ax.set_xlabel("x (Width)")
        ax.set_ylabel("z (Depth)")
        ax.set_zlabel("y (Height)")

        ax_stat.set_title('Panel')
        fig.subplots_adjust(top=0.96,bottom=0.03,left=0.025,right=0.925,hspace=0.16,wspace=0.12)
        ax_stat.set_axis_off()
        ax.view_init(elev=25, azim=135) # starting 3d view

        # drone
        self.agent_plt, = ax.plot(0,0,0, 'ko', markersize=6)
        self.motor_plts = [ax.plot(np.zeros(2),np.zeros(2),np.zeros(2), '-k', marker='o', mfc = 'red', mec = 'red', markersize=4, linewidth=1)[0] for _ in range(2)]
        self.drone_txt = ax_stat.text(0.02, 0.65, "t", va = "top", ha="left", transform = ax_stat.transAxes)
        ax.plot(*agent.init_state[:3],'.',color = 'gray', markersize=10) # origin
        self.ic_plt, = ax.plot(0,0,0, '.', color = 'gray',markersize=10) 
        self.ep_plt, = ax.plot(0,0,0, '.', color = 'gray',markersize=10)

        # target
        self.moth_plt, = ax.plot(1,0,0, 'kx', markersize=6)
        self.moth_txt = ax_stat.text(0.02, 0.55, "t", va = "top", ha="left", transform = ax_stat.transAxes)

        # planner
        self.traj_plt = ax.plot(np.zeros(20),np.zeros(20),np.zeros(20),  '--', color = 'gray', markersize=5, alpha=0.5)[0]
        self.planner_txt = ax_stat.text(0.02, 0.85, "t", va = "top", ha="left", transform = ax_stat.transAxes)
        self.planner_status_txt = ax_stat.text(0.27, 0.813, "t", color = 'g', va = "top", ha="left", transform = ax_stat.transAxes)

        # controller 
        self.x_pred_plts = ax.plot(np.zeros(self.controller.N), np.zeros(self.controller.N), np.zeros(self.controller.N), marker='', color = 'limegreen', linestyle='-',markersize=5, linewidth=3)[0]
        self.controller_txt = ax_stat.text(0.02, 0.75, "t", va="top", ha="left", transform = ax_stat.transAxes)

        # stats
        ax_stat.axline((0, 0), (0, 1), linewidth=2, alpha = 0.5, color='red', transform = ax_stat.transAxes)  # vline
        
        ax_stat.text(0.02, 0.95, "Status", weight='bold', ha="left", transform = ax_stat.transAxes)
        ax_stat.axline((0, 0.93), (1, 0.93), linewidth=2, alpha = 0.5, color='gray', transform = ax_stat.transAxes) # h- separator
        self.timer_txt = ax_stat.text(0.02, 0.9, "t", va = "top", ha="left", transform = ax_stat.transAxes)
        ax_stat.text(0.02, 0.45, "Controls", weight='bold', ha="left", transform = ax_stat.transAxes)
        ax_stat.axline((0, 0.43), (1, 0.43), linewidth=2, alpha = 0.5, color='gray', transform = ax_stat.transAxes)
        
        # plot static gefence planes
        x = [self.gf[0][1],self.gf[0][1], self.gf[0][1], self.gf[0][1]], [self.gf[0][0], self.gf[0][0], self.gf[0][0], self.gf[0][0]], [self.gf[0][1], self.gf[0][0], self.gf[0][0], self.gf[0][1]], [self.gf[0][1], self.gf[0][0],self.gf[0][0], self.gf[0][1]]
        y = [self.gf[2][1],self.gf[2][0], self.gf[2][0], self.gf[2][1]], [self.gf[2][1], self.gf[2][0], self.gf[2][0], self.gf[2][1]], [self.gf[2][1], self.gf[2][1], self.gf[2][1], self.gf[2][1]], [self.gf[2][0], self.gf[2][0],self.gf[2][0], self.gf[2][0]]
        z = [self.gf[1][1],self.gf[1][1], self.gf[1][0], self.gf[1][0]], [self.gf[1][1], self.gf[1][1], self.gf[1][0], self.gf[1][0]], [self.gf[1][1], self.gf[1][1], self.gf[1][0], self.gf[1][0]], [self.gf[1][1], self.gf[1][1],self.gf[1][0], self.gf[1][0]]
        # self.controller.set_geofence(self.gf)
        surfaces = []

        for i in range(len(x)):
            surfaces.append( [list(zip(x[i],y[i],z[i]))] )

        for surface in surfaces:
            ax.add_collection3d(Poly3DCollection(surface, alpha=0.2, color = 'gray'))

        np.random.seed(1)
        self.x_preds = np.zeros(0)
        self.replay = False

        # Replay a given log file from 'folder_path'
        if self.replay:
            host = "pats84"
            username = "pats"
            key_path = "/home/nikhil/.ssh/pats_wg_id_ed25519"
            key = paramiko.RSAKey(filename=key_path)
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(host, 22, username, pkey=key)
            sftp = ssh.open_sftp()

            folder_path = "/home/pats/pats/data/20231109_184421/"
            flight_num = 2

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
            self.ani_interval = 10
            # self.anim = animation.FuncAnimation(fig, self.replay_update, frames=len(data), interval=20, blit=True)
        
        # Dynamic simulation 
        else:
            num_frames = 200
            self.ani_interval = self.agent.dt*10

        # Could add trajectory planning inside a thread for dynamic planning
        # self.traj_thread = RepeatTimer(0.2, function = self.traj_update) 
        # self.traj_thread = threading.Thread(target = self.traj_update) 
        self.traj_update() # just one run to get started
        # self.traj_thread.start()
        
        # inits
        self.counter = 0
        self.caught = 0
        self.not_caught = 0
        self.control_rate = 0
        self.plan_rate = 0

        self.anim = animation.FuncAnimation(fig, self.update, frames=num_frames, interval=self.ani_interval, blit=True)
        plt.show()

    def simulate(self, x, u):
        pass

    def render_update(self, i):
        # ========= Animation updates
        
        # target
        self.moth_plt.set_data_3d(self.moth.state[0], self.moth.state[2], self.moth.state[1])

        # drone
        self.agent_plt.set_data_3d(self.sim_x[0], self.sim_x[2], self.sim_x[1])
        ## motors
        control_dir = unit_vector(self.action)
        hover_dir = np.array([0,1,0])
        R = scipy.spatial.transform.Rotation.from_quat(quat_from_v1_v2(hover_dir, control_dir))
        motor_pts_t = R.apply(self.agent.motor_pos) + np.array([self.sim_x[0], self.sim_x[2], self.sim_x[1]])

        # drone cross
        self.motor_plts[0].set_data_3d(motor_pts_t[0::2,0], motor_pts_t[0::2,1], motor_pts_t[0::2,2])
        self.motor_plts[1].set_data_3d(motor_pts_t[1::2,0], motor_pts_t[1::2,1], motor_pts_t[1::2,2])

        # controller
        try:
            self.x_pred_plts.set_data_3d(self.x_preds[:,0], self.x_preds[:,2], self.x_preds[:,1])
        except:
            pass


        # planner
        self.ic_plt.set_data_3d(self.x_traj[self.ic_idx, 0], self.x_traj[self.ic_idx, 2],self.x_traj[self.ic_idx, 1])
        self.ep_plt.set_data_3d(self.x_traj[-1, 0], self.x_traj[-1, 2],self.x_traj[-1, 1])
        try:
            self.traj_plt.set_data_3d(self.x_traj[:,0], self.x_traj[:,2], self.x_traj[:,1])
            pass
        except:
            pass

        # text info
        elapsed_time = self.counter*self.agent.dt
        self.timer_txt.set_text(f"Time: {elapsed_time:0.2f} seconds")
        self.planner_txt.set_text(f"Planner \n ------ \n status: \n cost: {self.opt_cost} \n rate: {self.plan_rate}")
        
        if self.opt_status == 0:
            self.planner_status_txt.set_text(f"Optimal")
            self.planner_status_txt.set_color('g')
        elif self.opt_status == 4:
            self.planner_status_txt.set_text(f"Weak infeasibility")
            self.planner_status_txt.set_color('y')
        else:
            self.planner_status_txt.set_text(f"Unknown")
            self.planner_status_txt.set_color('k')

        match self.controller.status :
            case 0:
                ctrl_text = "Success"
            case 1:
                ctrl_text = "Failed"
            case 2:
                ctrl_text = "Max. Iter"
            case 3:
                ctrl_text = "Min. Step"
            case 4:
                ctrl_text = "Failed (QP)"
            case _:
                ctrl_text = "unknown"

        self.controller_txt.set_text(f"Controller\n ------\n status: {ctrl_text} \n rate: {10*(self.control_rate//10)}")
        self.drone_txt.set_text(f"Drone\n ------\n status: {self.agent.status}\n caught: {self.caught}\n")
        self.moth_txt.set_text(f"Moth\n ------\n status: {self.moth.status} \n escaped: {self.not_caught}")
        
    def traj_update(self):
        start = time.time()

        die_moth_die = MothObliterator(drone_state = self.sim_x, moth_state = self.moth.init_state)
        x_opt, self.opt_cost, self.opt_status = die_moth_die.optimize()

        self.t1 = x_opt[0]
        mvajscp = die_moth_die.traj_opt

        dt = Dynamics.dt
        pos = mvajscp.evaluate(order = 0, dt=dt)
        vel = mvajscp.evaluate(order = 1, dt=dt)
        acc = mvajscp.evaluate(order = 2, dt=dt)
        self.traj = np.hstack([pos, vel, acc])
        self.x_traj = pos
        self.ic_idx = int(x_opt[0]//self.agent.dt)

        self.plan_time = (time.time() - start)

        # print("max_vel = ", np.max(np.linalg.norm(vel, axis=1)))
        # print("max_acc = ", np.max(np.linalg.norm(acc, axis=1)))
        # print(x_opt)

    def update(self, i):
        ani_start = time.time() 
        if self.replay:
            self.sim_x = self.pos_log[i]
            self.x_preds = self.preds_log[i]
            self.x_traj = self.traj_log[i]
        else:
            if i>0:   # because matplotlib takes a while to start and keeps the counter at 0 all the time
                self.moth.update()
                self.counter += 1
                self.x_preds = self.controller.preds
                
                # ======= calculate optimal control problem
                control_start = time.time()
                self.action = self.controller(state_c=self.sim_x, traj=self.traj, i=self.counter-1)
                self.control_rate = 1/(time.time()-control_start)
                self.sim_x = self._simulator.simulate(x = self.sim_x, u = self.action)

        ani_tuple = self.render_update(i)
        dist_to_moth = np.linalg.norm(self.sim_x[:3]-self.moth.state[:3])
         
        # ====== Sanity checks
        # reset simulation if you catch it or it goes out of bounds
        if dist_to_moth < 0.05: # 5 cm accuracy of catching
            self.reset()
            self.caught +=1

        if not in_arena(self.moth.state):
            self.reset()
            self.not_caught += 1

        return self.agent_plt, self.moth_plt, self.x_pred_plts, self.traj_plt, *self.motor_plts, self.timer_txt, self.planner_txt, self.planner_status_txt, self.drone_txt, self.controller_txt, self.moth_txt, self.ic_plt, self.ep_plt

    def reset(self):
        """Reset simulation and actors and plan again"""
        self.moth.randomize()
        self.sim_x = self.agent.reset()

        self.counter = 0
        # plan global trajectory (should ideally be dynamic)
        self.traj_update()
            
    def replay_update(self, i):
        ani_tuple = self.render_update()
        return self.agent_plt, *self.motor_plts, *self.x_pred_plts, *self.traj_plt

