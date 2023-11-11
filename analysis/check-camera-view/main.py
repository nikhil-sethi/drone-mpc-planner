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
from io import StringIO
from math import pi, sin, cos
from plotter import plot_frame, plot_normal_vectors
from calc_linalg import calc_planeNormVec, is_pointInVolume, get_orthogonalVector, get_distanceToPlane, get_distanceToPlanes, get_intersectionOf3Planes, get_hesseNormalForm, get_cornerPoints
from utils import get_debug_data, plot_corner_points

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


def cleanWhitespaces(filepath):
    file = open(filepath, "r")
    datastring = file.read()
    file.close()
    datastring = datastring.replace(' ', '')
    datastring = StringIO(datastring)
    return datastring

# SETUP SCENARIO:
planes, corner_points = get_debug_data(r'volume_log.txt')


# Plot the frame of the viewable area
colors = plt.rcParams['axes.prop_cycle'].by_key()['color']
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
fig.suptitle('Volume definition')
plot_corner_points(ax, corner_points)

ax.legend()
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.view_init(elev=105, azim=-90)
#ax.legend()

plt.show()
