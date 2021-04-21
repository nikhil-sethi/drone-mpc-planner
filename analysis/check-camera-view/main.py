#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  6 14:50:55 2019

@author: ludwig

    %matplotlib qt
    %matplotlib inline
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import pi, sin, cos
from plotter import plot_frame, plot_normal_vectors
from calc_linalg import calc_planeNormVec, is_pointInVolume, get_orthogonalVector, get_distanceToPlane, get_distanceToPlanes, get_intersectionOf3Planes, get_hesseNormalForm, get_cornerPoints

# INDEX DEFINTIONS:
x = 0
y = 1
z = 2
p0 = 0
n = 1

front = 0
top = 1
left = 2
right = 3
bottom = 4
back = 5
camera = 6

def get_view_planes(filepath=None):
    n_front = np.array([[0, 0.43420371, -0.90081471]]).T
    n_top = np.array([[0, -0.99650627, 0.083518513]]).T
    n_left = np.array([[-0.70126724, -0.40229687, -0.58854181]]).T
    n_right = np.array([[0.70580268, -0.39976308, -0.58483499]]).T
    n_bottom = np.array([[0, 1, 0]]).T
    n_back = np.array([[0, 0, 1]]).T
    n_camera = np.array([[0, -0.56431156, -0.82556188]]).T

    p0_front = np.array([[0,0,0]]).T
    p0_top = np.array([[0,0,0]]).T
    p0_left = np.array([[0,0,0]]).T
    p0_right = np.array([[0,0,0]]).T
    p0_bottom = np.array([[0, -1.413, 0]]).T
    p0_back = np.array([[0, 0, -10]]).T
    p0_camera = np.array([[0, -0.47966483, -0.70172763]]).T

    if(filepath):
        file = open(filepath, "r")
        datastring = file.read()
        file.close()

        lines = datastring.split('\n')
        for line in lines:
            words = line.split(' ')
            if words[0]=='Cameraview>p0_front:':
                p0_front = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_front:':
                n_front = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>p0_top:':
                p0_top = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_top:':
                n_top = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>p0_left:':
                p0_left = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_left:':
                n_left = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>p0_right:':
                p0_right = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_right:':
                n_right = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>p0_bottom:':
                p0_bottom = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_bottom:':
                n_bottom = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>p0_back:':
                p0_back = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_back:':
                n_back = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>p0_camera:':
                p0_camera = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])
            elif words[0]=='Cameraview>n_camera:':
                n_camera = np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])

    plane_front = [p0_front, n_front]
    plane_top = [p0_top, n_top]
    plane_left = [p0_left, n_left]
    plane_right = [p0_right, n_right]
    plane_bottom = [p0_bottom, n_bottom]
    plane_back = [p0_back, n_back]
    plane_camera = [p0_camera, n_camera]

    planes = [plane_front, plane_top, plane_left, plane_right, plane_bottom, plane_back, plane_camera]
    plane_names = ['plane_front', 'plane_top', 'plane_left', 'plane_right', 'plane_bottom', 'plane_back', 'plane_camera']

    return planes, plane_names


def get_drone_blink_location(filepath=None):
    drone_state = np.array([[-0.00701305, -1.2776, -2.517]]).T

    if filepath:
        file = open(filepath, "r")
        datastring = file.read()
        file.close()
        lines = datastring.split('\n')
        for line in lines:
            words = line.split(' ')
            if(words[0]=='Blink-drone-location:'):
                return np.array(eval(words[1]+words[2]+words[3])).reshape([3,1])

    return drone_state

    
if __name__ == "__main__":

    # SETUP SCENARIO:
    drone_location = np.array([[1.02554, -1.3925,-2.3399]]).T #get_drone_blink_location(r'volume_log.txt')
    drone_state = [drone_location, np.array([[-0, -1, 0]]).T]
    planes, plane_names = get_view_planes(r'volume_log.txt')

    # Plot the frame of the viewable area
    colors = ['r', 'b', 'g', 'orange', 'purple', 'yellow']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.suptitle('Volume definition')
    xcamera, xfrd, xfld, xbld, xbrd, xblu, xbru, xflu, xfru, xflm, xfrm = get_cornerPoints(planes)
    plot_frame(ax, xcamera, xfrd, xfld, xbld, xbrd, xblu, xbru, xflu, xfru, xflm, xfrm, 'k')
    plot_normal_vectors(ax, planes[front][n], planes[top][n], planes[left][n], planes[right][n], planes[bottom][n], planes[back][n], colors)
    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.view_init(elev=105, azim=-90)

    # Check if point is in view:
#    original = np.array([[-4,-1.73,-2]]).T
#    fixed = np.array([[-2.23553, -1.42678, -2.15853]]).T
    point = drone_state[0]
    print(is_pointInVolume(drone_state[0], planes))
    ax.plot([drone_state[0][0,0]],[drone_state[0][1,0]],[drone_state[0][2,0]],'ko')

    # Check distance to frames:
    dist, intersection_point, plane_idx = get_distanceToPlanes(drone_state, planes)
    print('min dist to plane', plane_names[plane_idx], 'with distance', dist, 'm.')

    ax.plot([drone_state[0][0, 0]], [drone_state[0][1, 0]], [drone_state[0][2, 0]], 'bx', label='drone_position')
    ax.plot([drone_state[0][0, 0], drone_state[0][0, 0]+drone_state[1][0, 0]],
            [drone_state[0][1, 0], drone_state[0][1, 0]+drone_state[1][1, 0]],
            [drone_state[0][2, 0], drone_state[0][2, 0]+drone_state[1][2, 0]], 'b', label='drone_velocity')
    ax.plot([intersection_point[0, 0]], [intersection_point[1, 0]], [intersection_point[2, 0]], 'rx', label='intersection point')
    ax.plot([0., 0.], [0., -sin(40/180*pi)*2], [0., -cos(40/180*pi)*2], 'k')
    ax.plot([0.], [-sin(40/180*pi)*1], [-cos(40/180*pi)*1], 'kx')
    ax.legend()
    plt.show()
#
#    disttop, isptop = get_distanceToPlane(drone_state, planes[top])
#    print('min dist to plane',plane_names[top],'with distance',disttop,'m.')
