import numpy as np
import enum

class LogLevel(enum.IntEnum):
    NONE = 0
    INFO = 1
    DEBUG = 2
    ERROR = 3

class Order(enum.IntEnum):
    POS = 0
    VEL = 1
    ACC = 2
    JERK = 3
    SNAP = 4

geofence = np.array([(-1, 1), (-1.3, 0), (-2.95, 0)])
n_dims = 3
n_vars = 5

class Dynamics:
    AMIN = 5 # minimum acceleration for the norm
    AMAX = 20 # max acceleration for the norm
    VMAX = 4
    VYMIN = 0
    VMIN = -10000 
    G = 9.81
    dt = 0.01

class MPCParams:
    N = 10

class Interception:
    PHI = (0,np.pi)
    THETA = (-np.pi/2, np.pi/2)
    MAG = (Dynamics.AMIN, Dynamics.AMAX-5)

class Weights:
    SmoothSeg = [0.01, 0.001]
    MatchAccIC = 3
    MaxAccIC = 0.1
    MinTime = [2, 2]

