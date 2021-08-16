#!/usr/bin/env python3
import numpy as np

def update_mode(words, mode):
    if words[0] == "FlightAreaConfiguration:":
        return "planes"
    elif words[0] == "CornerPoints:":
        return "corner_points"
    else:
        return mode

def eval_plane_line(words):
    if words[0] == "Plane:":
        support = np.array(eval(words[6] + words[7] + words[8][:-1])) 
        normal =  np.array(eval(words[10] + words[11] + words[12][:-1])) 
        return [support, normal]
    else:
         return np.nan

def eval_corner_point_line(words):
    if words[0] == "CornerPoint:":
        return np.array(eval(words[2] + words[3] + words[4]))
    elif words[0] == "Plane:":
        return "new_plane_detected"
    else:
        return "None"

def get_debug_data(filepath):
    planes = []
    corner_points = []
    corner_points_of_plane = []
    mode = "undefined"
    if filepath:
        file = open(filepath, "r")
        datastring = file.read()
        file.close()
        lines = datastring.split('\n')

        for line in lines:
            words = line.split(' ')
            mode = update_mode(words, mode)

            if mode == "planes":
                pln = eval_plane_line(words)
                if not np.isnan(pln).any():
                    planes.append(pln)

            elif mode == "corner_points":
                crnr_pnt = eval_corner_point_line(words)
                if crnr_pnt == "new_plane_detected":
                    if len(corner_points_of_plane) > 0:
                        corner_points.append(corner_points_of_plane)
                    corner_points_of_plane = []
                elif crnr_pnt == "None":
                    pass
                else:
                    corner_points_of_plane.append(crnr_pnt)

    return planes, corner_points


def plot_corner_points(ax, corner_points):
    
    for crnr_pnts_of_plane in corner_points:
        x = []
        y = []
        z = []

        for crnr_pnt in crnr_pnts_of_plane:
            x.append(crnr_pnt[0])
            y.append(crnr_pnt[1])
            z.append(crnr_pnt[2])

        ax.plot(x, y, z)




