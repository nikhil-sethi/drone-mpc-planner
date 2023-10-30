import numpy as np

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

