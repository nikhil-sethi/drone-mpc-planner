#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""."""

"""
Camera roll: 0.14°- max: 0.7°. Pitch: 31.32°
Cameraview>p0_front: [0, 0, 0]
Cameraview>n_front: [0, 0.49303865, -0.8700074]
Cameraview>p0_top: [0, 0, 0]
Cameraview>n_top: [0, -0.99962962, 0.027213788]
Cameraview>p0_left: [0, 0, 0]
Cameraview>n_left: [-0.70986247, -0.36617854, -0.60167152]
Cameraview>p0_right: [0, 0, 0]
Cameraview>n_right: [0.70545954, -0.36847126, -0.60543853]
Cameraview>p0_bottom: [0, -2.0659051, 0]
Cameraview>n_bottom: [0, 1, 0]
Cameraview>p0_back: [0, 0, -4.0765491]
Cameraview>n_back: [0, 0, 1]
Cameraview>p0_camera: [0, -0.44190535, -0.72609895]
Cameraview>n_camera: [0, -0.51988864, -0.85423404]
"""
import numpy as np
n_plane_parameters = 6
n_planes = 10
idx_support = [0, 1, 2]
idx_normal = [3, 4, 5]

def plane_parameters():
    """."""
    cubesize = 3
    planes = np.array([[ cubesize/2, 0, 0,  -1, 0, 0], #left
                       [-cubesize/2, 0, 0,   1, 0, 0], #right
                       [0, 0, 0, 0, 0, -1], #front
                       [0, 0, -cubesize, 0, 0, 1], #back
                       [0, 0, 0, 0, -1, 0], #top
                       [0, 0, 0, 0, -1, 0], #top
                       [0, 0, 0, 0, -1, 0], #top
                       [0, -cubesize, 0, 0, 1, 0]]) #bottom
    return planes
