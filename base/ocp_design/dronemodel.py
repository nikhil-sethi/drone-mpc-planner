#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from numpy import sqrt
import casadi

# State Vector Indices
idx_posx = 0
idx_posy = 1
idx_posz = 2
idx_velx = 3
idx_vely = 4
idx_velz = 5

idx_pos = [0, 1, 2]
idx_vel = [3, 4, 5]

# Control Vector Indices
idx_accx = 0
idx_accy = 1
idx_accz = 2

idx_acc = [0, 1, 2]

## Model Size
n_states = 6
n_inputs = 3

DRONE_MAX_THRUST = 20 #[m/s^2]

def drone_dynamics(X, U):
    PosX = X[:, idx_posx]
    PosY = X[:, idx_posy]
    PosZ = X[:, idx_posz]
    VelX = X[:, idx_velx]
    VelY = X[:, idx_vely]
    VelZ = X[:, idx_velz]

    AccX = U[:, idx_accx]
    AccY = U[:, idx_accy]
    AccZ = U[:, idx_accz]

    # Model parameters
    dxdt = 0*X
    dxdt[:, idx_posx] = VelX
    dxdt[:, idx_posy] = VelY
    dxdt[:, idx_posz] = VelZ
    dxdt[:, idx_velx] = AccX
    dxdt[:, idx_vely] = AccY
    dxdt[:, idx_velz] = AccZ
    return dxdt

