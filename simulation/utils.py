import numpy as np
from conf import geofence
from scipy import optimize
def rot_quat(vec1):
    vec0 = np.array([0,0,1])
    dot = vec0.dot(vec1)
    if (dot < -0.999999):
        ret = np.array([0, 0, -1, 0])
    else:
        ret = np.zeros(4)
        ret[0] = 1 + dot
        ret[1:] = np.cross(vec0, vec1)
        ret /= np.linalg.norm(ret)
        return ret

def fact(j, i):
    """factorial of j upto i. Used for derivatives"""
    if i==0:
        return 1
    return j*fact(j-1, i-1)

def pow(n, k):
    if k<0:
        return 1
    return n**k

def unit_vector(vec):
    return vec/np.linalg.norm(vec)

def in_arena(point):
    if (geofence[0][0] <= point[0] <= geofence[0][1]) and (geofence[1][0] <= point[1] <= geofence[1][1]) and (geofence[2][0] <= point[2] <= geofence[0][1]):
        return True 
    return False

def sample_in_range(rng):
    # gf = np.array(geofence)
    sampled_pt =  rng[0] + (rng[1] - rng[0])*np.random.rand(3)

    return sampled_pt

def quat_from_v1_v2(v1, v2):
    """Return the shortes arc quaternion to rotate vector v1 to vector v2"""
    quat_ax = np.cross(v1, v2)
    quat_mag = np.sqrt(np.linalg.norm(v1)**2 + np.linalg.norm(v2)**2)  + np.dot(v1, v2)
    return quat_ax[0], quat_ax[1], quat_ax[2], quat_mag
    