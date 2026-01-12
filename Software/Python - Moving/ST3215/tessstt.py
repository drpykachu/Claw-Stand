import sys
import pyvista as pv
from pyvistaqt import QtInteractor
import trimesh
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import pyautogui  # to automatically detect screen size


def circle(r, h, degree):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(np.deg2rad(degree))
    x = r * np.cos(np.deg2rad(degree))
    return [x, y, z]

def pathing(points, delta_Z,num_fingers,R_tar, H_tar):
    """Builds the path for the fingertip travel."""
    exponent = 4
    num_fingers_p1 = num_fingers+1
    
    class PointCount(Exception):
        """Checks if the amount of points is even."""
        pass

    if points % num_fingers != 0:
        raise PointCount(f"Number of points must be divisible by num_fingers ({num_fingers}).")

    # distributing points
    point_upper = int(points/num_fingers*(num_fingers-1))
    point_lower = int(points/num_fingers)

    # building top path
    top_path = circle(R_tar, H_tar, np.linspace(-360/num_fingers_p1/2, 360/num_fingers_p1/2, point_upper))
    top_path[2] = np.ones(len(top_path[0]))*top_path[2]
    
    # building bottom path
    print(point_lower)
    linearspace = np.linspace(0, 1, int(point_lower/2))
    expospace = linearspace**0.5
    expospace = np.concatenate((expospace,-expospace))
    bot_path_circ = circle(R_tar, H_tar, -360/num_fingers_p1/2*expospace)    
    
    y_points = bot_path_circ[1]
    bot_func = delta_Z*np.flip(y_points)**exponent / np.max(y_points**exponent)  + (bot_path_circ[2] - delta_Z)
    bot_path = [np.flip(bot_path_circ[0]),np.flip(bot_path_circ[1]),bot_func]    
     
    total_path = np.concatenate((top_path, bot_path), axis=1)    
    
    return total_path

# Digit Lengthsw


num_fingers = 5

R_tar       = 80  # Radius of target circle path 
H_tar       = 200  # Height of target circle path

delta_Z     = 10   # Dropping from path for reset 
points      = 200  # Number of points for path

plt.figure()
fingertip_path = pathing(points, delta_Z,num_fingers,R_tar, H_tar)    # Builds a fingertip path
plt.plot(fingertip_path[1],fingertip_path[2],'.')


plt.show()

