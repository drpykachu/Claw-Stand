import sys
import matplotlib
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg



def circle(r, h, degree):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(np.deg2rad(degree))
    x = r * np.cos(np.deg2rad(degree))
    return [x, y, z]


def path(points, delta_Z):
    """Builds the path for the fingertip travel. Top pathis set to portion of circle, bottom path set to an exponential function"""
    exponent = 4
    
    class PointCount(Exception):
        """Checks if the amount of points is even."""
        pass

    if points % 2 != 0:
        raise PointCount("Number of points must be even.")

    point_upper = int(points/2)
    point_lower = int(points/2)

    top_path = circle(R_tar, H_tar, np.linspace(-360/5/2, 360/5/2, point_upper))
    top_path[2] = np.ones(len(top_path[0]))*top_path[2]


    bot_func = delta_Z*np.flip(top_path[1])**exponent / np.max(top_path[1]**exponent)  + (top_path[2] - delta_Z)
    bot_path = [np.flip(top_path[0]),np.flip(top_path[1]),bot_func]    
        
    total_path = np.concatenate((top_path, bot_path), axis=1)

    return total_path
    
R_tar = 35
H_tar = 35



fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection="3d")
# ax.view_init(elev=90, azim=0) # X-Y plane initial view
# ax.view_init(elev=0, azim=0) # Y-Z plane initial view
# ax.view_init(elev=0, azim=-90) # X-Z plane initial view
ax.view_init(elev=15, azim=-45) # Trimetric inital view

# Sets axis labels
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")


circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, 101))
lims = 75
ax.set_xlim([-lims, lims])
ax.set_ylim([-lims, lims])
ax.set_zlim([-lims/10, lims])

# Plots red circular trajectory
ax.plot(circle_tar[0], circle_tar[1], circle_tar[2], 'k--', linewidth=1.5, label="Target Path")


############## path function heart ############################
    
######################################################################

# plots path
points = 100
delta_Z = 10
total_path  = path(points, delta_Z)
ax.plot(total_path[0], total_path[1], total_path[2], 'r', linewidth=1.5, label="Target Path")


# Plots directional rose
ax.plot([-lims, lims], [0, 0], [0, 0], '--', c = 'k', lw = 2,alpha = 0.2)
ax.plot([0, 0], [-lims, lims], [0, 0], '--', c = 'k', lw = 2,alpha = 0.2)
ax.plot([0, 0], [0, 0], [-lims/10, lims], '--', c = 'k', lw = 2,alpha = 0.2)


plt.show()