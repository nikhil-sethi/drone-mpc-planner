#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

# State Vector Indices
idx_insectposx = 0
idx_insectposy = 1
idx_insectposz = 2
idx_insectvelx = 3
idx_insectvely = 4
idx_insectvelz = 5

idx_insectpos = [0, 1, 2]
idx_insectvel = [3, 4, 5]

insect_model_idxs = {}
insect_model_idxs["INSECT_POSX"] = 0
insect_model_idxs["INSECT_POSY"] = 1
insect_model_idxs["INSECT_POSZ"] = 2
insect_model_idxs["INSECT_VELX"] = 3
insect_model_idxs["INSECT_VELY"] = 4
insect_model_idxs["INSECT_VELZ"] = 5

## Model Size
n_insect_states = 6

def insect_dynamics(X):
    PosX = X[:, idx_insectposx]
    PosY = X[:, idx_insectposy]
    PosZ = X[:, idx_insectposz]
    VelX = X[:, idx_insectvelx]
    VelY = X[:, idx_insectvely]
    VelZ = X[:, idx_insectvelz]

    # Model parameters
    dxdt = 0*X
    dxdt[:,idx_insectposx] = VelX
    dxdt[:,idx_insectposy] = VelY
    dxdt[:,idx_insectposz] = VelZ
    dxdt[:,idx_insectvelx] = 0
    dxdt[:,idx_insectvely] = 0
    dxdt[:,idx_insectvelz] = 0
    return dxdt
