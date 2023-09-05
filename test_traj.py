from trajectory import MinVelAccJerkSnapCrackPop
import numpy as np
import matplotlib.pyplot as plt

wps = np.array([
    [0., 1., 1.8944],
    [0., 2., 3.7888],
    [0,0,0]
]).T

tf = 8
dt = 0.05
n = int(tf/dt)
mvajscp = MinVelAccJerkSnapCrackPop(order=2, waypoints=wps.T, time=tf)
pos = mvajscp.optimize(plan_order = 0, num_pts=n)
vel = mvajscp.optimize(plan_order = 1, num_pts=n)
acc = mvajscp.optimize(plan_order = 2, num_pts=n)
print(pos)
print(vel)
print(acc)
plt.figure()
mvajscp.plot(pos)
mvajscp.plot_vec(pos, vel, color='gray')    
plt.show()