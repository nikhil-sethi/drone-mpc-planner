#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


def calc_planeNormVec(x1, x2, x3):
    """Calculates based on the 3 points in 3d a plane,
    which is defined as norm vector in the space p(n_,p0_). """
    vec_a = x2 - x1
    vec_b = x3 - x1

    n = np.cross(vec_a, vec_b)
    n = n / np.linalg.norm(n)

    p0 = x1

    return [p0, n]


def is_pointInVolume(point, planes):
    for i in range(len(planes)):
        dot_prd = np.dot(np.transpose(point - planes[i][0]), planes[i][1])
        if(dot_prd < 0):
            return False

    return True


def get_orthogonalVector(n):
    if(n[2] != 0):
        return np.array([[1, 1, (-n[0] - n[1]) / n[2]]], float).T
    elif(n[1] != 0):
        return np.array([[1, (-n[0] - n[2]) / n[1], 1]], float).T
    else:
        return np.array([[(-n[1] - n[2]) / n[0], 1, 1]], float).T


def get_distanceToPlane(vector, plane):
    """ Calculates the distance along the vector till the plane is hit.
    """
    b = (vector[0] - plane[0]).T

    n = np.reshape(plane[1], [3, 1])
    e1 = get_orthogonalVector(n)
    e2 = np.cross(n.T, e1.T)

    A = np.zeros([3, 3])
    A[:, 0] = e1.T
    A[:, 1] = e2
    A[:, 2] = -vector[1].reshape([3])

    if np.abs(np.linalg.det(A)) < 0.001:
        no_intersection = np.zeros([3, 1])
        no_intersection[:, :] = np.nan
        return np.inf, no_intersection

    unknowns = np.linalg.inv(A).dot(b.reshape([3]))

    dist = np.sign(unknowns[2]) * np.linalg.norm(unknowns[2] * vector[1])
    intersection_point = vector[0] + unknowns[2] * vector[1]

    return dist, intersection_point


def get_distanceToPlanes(vector, planes):
    min_idx = -1
    min_dist = np.inf
    min_intersection_point = np.zeros([3, 1])

    for i in range(len(planes)):
        dist, intersection_point = get_distanceToPlane(vector, planes[i])

        if(dist >= 0 and dist < min_dist):
            min_idx = i
            min_dist = dist
            min_intersection_point = intersection_point

    return min_dist, min_intersection_point, min_idx


def get_intersectionOf3Planes(plane1, plane2, plane3):
    d1, n01 = get_hesseNormalForm(plane1)
    d2, n02 = get_hesseNormalForm(plane2)
    d3, n03 = get_hesseNormalForm(plane3)

    b = np.array([d1, d2, d3]).reshape([3, 1])
    A = np.array([n01.T, n02.T, n03.T]).reshape([3, 3])

    return np.linalg.inv(A).dot(b)


def get_hesseNormalForm(plane_normal_form):
    n0 = plane_normal_form[1] * np.linalg.norm(plane_normal_form[1])
    d = plane_normal_form[0].T.dot(n0)

    return d, n0


def get_cornerPoints(planes):
    front = 0
    top = 1
    left = 2
    right = 3
    bottom = 4
    back = 5
    camera = 6

    xcamera = get_intersectionOf3Planes(planes[front], planes[top], planes[left])
    xfrd = get_intersectionOf3Planes(planes[front], planes[right], planes[bottom])
    xfld = get_intersectionOf3Planes(planes[front], planes[left], planes[bottom])
    xbld = get_intersectionOf3Planes(planes[back], planes[left], planes[bottom])
    xbrd = get_intersectionOf3Planes(planes[back], planes[right], planes[bottom])
    xblu = get_intersectionOf3Planes(planes[back], planes[left], planes[top])
    xbru = get_intersectionOf3Planes(planes[back], planes[right], planes[top])
    xflu = get_intersectionOf3Planes(planes[camera], planes[left], planes[top])
    xfru = get_intersectionOf3Planes(planes[camera], planes[right], planes[top])
    xflm = get_intersectionOf3Planes(planes[camera], planes[front], planes[left])
    xfrm = get_intersectionOf3Planes(planes[camera], planes[front], planes[right])

    return xcamera, xfrd, xfld, xbld, xbrd, xblu, xbru, xflu, xfru, xflm, xfrm


if __name__ == "__main__":
    pass
