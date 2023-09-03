import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter



# x, y, z, vx, vy, vz
data = np.loadtxt("log_flight6.csv", delimiter=";", skiprows=1, usecols=(12, 13, 14, 19, 20,21, 23, 24, 25, 50,51,52, 31)) 

pos = data[:,:3]
vel = data[:,3:6]
acc = data[:,6:9]
control = data[:, 9:12]
timesteps = data[:, 12]
time = np.cumsum(timesteps)
dt = np.mean(timesteps)


# Calculate velocity using the Savitzky-Golay filter
window_size = 11  # Adjust the window size as needed
order = 2  # Polynomial order
vel_est = savgol_filter(pos[:,0], window_size, order, deriv=1, delta=dt)
acc_est = savgol_filter(vel[:,0], window_size, order, deriv=1, delta=dt)
print(time.shape)
v_mag = np.sqrt(np.sum(vel**2, axis=1))
max_speed = max(v_mag)

plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(time, pos[:,0], label='Position')
plt.xlabel('Time')
plt.ylabel('Position')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(time, vel[:,0], label='Velocity GT', color='blue')
plt.plot(time, vel_est, label='Velocity est', color='orange')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(time, acc[:,0], label='Velocity GT', color='blue')
plt.plot(time, acc_est, label='Velocity est', color='orange')
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.legend()

plt.tight_layout()
plt.show()

# training a network

from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler  


X = np.hstack([pos, vel, control])
Y = np.hstack([vel, acc])

X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.25, random_state=42)

# transformations
scaler = StandardScaler()  
scaler.fit(X_train)  
X_train = scaler.transform(X_train)
X_test = scaler.transform(X_test)

clf = MLPRegressor(solver='adam', alpha=1e-6, activation='relu', hidden_layer_sizes=(100, 50), random_state=4, max_iter=550, warm_start=True, verbose=True)

clf.fit(X_train, Y_train)

import pickle
clf = pickle.load(open('clf', 'rb'))

pred = clf.predict(X_test)

# print(X)
# print(Y_test)
plt.figure()
plt.plot(Y_test[:,5], 'b-')
plt.plot(pred[:,5], 'r-')
plt.show()
# print(np.sum(pred-Y_test))


# save model

with open("clf", 'wb') as f:
    pickle.dump(clf, f)