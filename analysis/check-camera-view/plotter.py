#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def plot_frame(ax, xcamera, xfrd, xfld, xbld, xbrd, xblu, xbru, xflu, xfru, xflm, xfrm, linestyle):
    x = 0
    y = 1
    z = 2
    ax.plot([xblu[x, 0], xbru[x, 0], xbrd[x, 0], xbld[x, 0], xfld[x, 0], xfrd[x, 0], xbrd[x, 0]],
            [xblu[y, 0], xbru[y, 0], xbrd[y, 0], xbld[y, 0], xfld[y, 0], xfrd[y, 0], xbrd[y, 0]],
            [xblu[z, 0], xbru[z, 0], xbrd[z, 0], xbld[z, 0], xfld[z, 0], xfrd[z, 0], xbrd[z, 0]], linestyle)
    ax.plot([xbru[x, 0], xfru[x, 0], xflu[x, 0], xblu[x, 0], xbld[x, 0]],
            [xbru[y, 0], xfru[y, 0], xflu[y, 0], xblu[y, 0], xbld[y, 0]],
            [xbru[z, 0], xfru[z, 0], xflu[z, 0], xblu[z, 0], xbld[z, 0]], linestyle)
    ax.plot([xfld[x, 0], xflm[x, 0], xflu[x, 0], xfru[x, 0], xfrm[x, 0], xfrd[x, 0]],
            [xfld[y, 0], xflm[y, 0], xflu[y, 0], xfru[y, 0], xfrm[y, 0], xfrd[y, 0]],
            [xfld[z, 0], xflm[z, 0], xflu[z, 0], xfru[z, 0], xfrm[z, 0], xfrd[z, 0]], linestyle)
    ax.plot([xfrm[x, 0], xflm[x, 0]],
            [xfrm[y, 0], xflm[y, 0]],
            [xfrm[z, 0], xflm[z, 0]])


def plot_normal_vectors(ax, planes, plane_names, colors):

    for i, plane in enumerate(planes):
        if plane_names[i]!="plane_left" and plane_names[i]!="plane_right":
            ax.plot([0, plane[1][0, 0]],
                    [0, plane[1][1, 0]],
                    [0, plane[1][2, 0]],
                    color=colors[i],
                    label=plane_names[i])
        else: 
            ax.plot([plane[0][0, 0], plane[0][0, 0] + plane[1][0, 0]],
                    [0, plane[1][1, 0]],
                    [0, plane[1][2, 0]],
                    color=colors[i],
                    label=plane_names[i])


def plot_plane_characteristics(ax, planes, colors):
    p0 = 0
    n = 1
    x = 0
    y = 1
    z = 2

    for i, plane in enumerate(planes):
        ax.plot([plane[p0][x], plane[p0][x] + plane[n][x]],
                [plane[p0][y], plane[p0][y] + plane[n][y]],
                [plane[p0][z], plane[p0][z] + plane[n][z]], color=colors[i])


if __name__ == "__main__":
    pass
