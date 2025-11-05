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

# ================= Functions ===================

def solve_thetas(Zp, Yp, Xp, A, B, C, T, Offset_R):
    """Returns the angles (in radians) of motors to obtain a point in space (Xp, Yp, Zp)"""
    Xp = Xp - Offset_R
    Yp = Yp

    # Helps with breakdown of atan2 at infinity as Yp -> 0
    if Yp < 0.0005 and Yp > 0:
        Yp = 0.0005        
    if Yp > -0.0005 and Yp <= 0:
        Yp = -0.0005
        
    theta_a = np.atan2((Zp-A),Yp)
    ca = np.cos(theta_a)
    
    S = Yp/ca - B
    R = np.hypot(Xp, S)           # sqrt(Xp^2 + S^2)
    phi = np.arctan2(S, Xp)
    K = (R*R + C*C - T*T) / (2.0*C)

    if abs(K/R) > 1.0 + 1e-12:
        return []  # no real solutions

    # clamp for numeric stability
    val = np.clip(K/R, -1.0, 1.0)
    acos_val = np.arccos(val)

    thetab_solutions = [phi + acos_val, phi - acos_val]
    solutions = []
    for tb in thetab_solutions:
        cb = np.cos(tb); sb = np.sin(tb)
        cd = (Xp - C*cb) / T
        sd = (S  - C*sb) / T
        # numeric clamp
        cd = np.clip(cd, -1.0, 1.0)
        sd = np.clip(sd, -1.0, 1.0)
        td = np.arctan2(sd, cd)

        # ensures only one solution
        tc = td - tb

        if tc > 0:
            solutions.append((theta_a, tb, td))
            
    return solutions


def point_coords(theta_a,theta_b,theta_c, Offset_R, A, B, C, T):
    """Return 3D coordinates of finger joints for a given circle degree."""
    theta_d = theta_b - theta_c
    
    # bottom - cyan
    x0 = 0 + Offset_R
    y0 = 0
    z0 = 0 

    # Black dot
    x1 = x0
    y1 = y0
    z1 = z0 + A 

    # Pink dot
    x2 = x1
    y2 = y1 + B*np.cos(theta_a)
    z2 = z1 + B*np.sin(theta_a)

    # Green dot
    x3 = x2 + C*np.cos(theta_b)
    y3 = y2 + C*np.sin(theta_b)*np.cos(theta_a)
    z3 = z2 + C*np.sin(theta_b)*np.sin(theta_a)
    
    # Red dot
    x4 = x3 + T*np.cos(theta_c)
    y4 = y3 + T*np.sin(theta_c)*np.cos(theta_a)
    z4 = z3 + T*np.sin(theta_c)*np.sin(theta_a)

    return np.array([
        [x0, x1, x2, x3, x4],
        [y0, y1, y2, y3, y4],
        [z0, z1, z2, z3, z4]
    ])


def circle(r, h, degree):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(np.deg2rad(degree))
    x = r * np.cos(np.deg2rad(degree))
    return [x, y, z]


def pathing(points, delta_Z,num_fingers,R_tar, H_tar):
    """Builds the path for the fingertip travel."""
    exponent = 6
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
    bot_path_circ = circle(R_tar, H_tar, np.linspace(-360/num_fingers_p1/2, 360/num_fingers_p1/2, point_lower))    
    bot_func = delta_Z*np.flip(bot_path_circ[1])**exponent / np.max(bot_path_circ[1]**exponent)  + (bot_path_circ[2] - delta_Z)
    bot_path = [np.flip(bot_path_circ[0]),np.flip(bot_path_circ[1]),bot_func]    
     
    total_path = np.concatenate((top_path, bot_path), axis=1)
    return total_path


def rotate_point(origin, point, degree):
    ox, oy = origin
    px, py = point
    qx = ox + np.cos(np.deg2rad(degree)) * (px - ox) - np.sin(np.deg2rad(degree)) * (py - oy)
    qy = oy + np.sin(np.deg2rad(degree)) * (px - ox) + np.cos(np.deg2rad(degree)) * (py - oy)
    return qx, qy


def rotate_vector(vec, degree, origin=(0, 0)):
    ox, oy = origin
    theta = np.deg2rad(degree)
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])
    XY = vec[:2, :]
    XY_shifted = XY - np.array([[ox], [oy]])
    XY_rot = R @ XY_shifted + np.array([[ox], [oy]])
    if vec.shape[0] == 3:
        rotated = np.vstack((XY_rot, vec[2, :]))
    else:
        rotated = XY_rot
    return rotated

# ==================== Pyvista functions 
def camera_distance(plotter, distance):
    """ Function for setting the camera distance. """
    cam = plotter.camera
    pos = np.array(cam.position)       # where the camera is
    focal = np.array(cam.focal_point)  # where it's looking
    direction = pos - focal
    direction = direction / np.linalg.norm(direction)  # normalize
    new_pos = focal + direction * distance
    plotter.camera.position = new_pos
    


def translate_object(pv_object, translation_vector):

    translation_vector = np.array(translation_vector)
    pv_object.points += translation_vector

def rotate_around_line(pv_object, point_on_line, line_vector, angle_deg):
    """
    Rotates a PyVista mesh or point set around an arbitrary line.

    Parameters
    ----------
    pv_object : pv.PolyData
        The PyVista object to rotate.
    point_on_line : array-like of shape (3,)
        A point on the rotation axis.
    line_vector : array-like of shape (3,)
        The direction vector of the rotation axis.
    angle_deg : float
        The rotation angle in degrees.

    Returns
    -------
    None
        The pv_object is updated in-place.
    """
    # Convert to numpy arrays
    point_on_line = np.array(point_on_line, dtype=float)
    line_vector = np.array(line_vector, dtype=float)
    
    # Normalize axis vector
    axis = line_vector / np.linalg.norm(line_vector)
    
    # Translate points so the rotation axis passes through origin
    points_shifted = pv_object.points - point_on_line

    # Rodrigues' rotation formula
    theta = np.deg2rad(angle_deg)
    kx, ky, kz = axis
    K = np.array([[0, -kz, ky],
                  [kz, 0, -kx],
                  [-ky, kx, 0]])
    R = np.eye(3) + np.sin(theta)*K + (1 - np.cos(theta))*(K @ K)

    # Apply rotation
    points_rotated = points_shifted @ R.T

    # Translate back
    pv_object.points = points_rotated + point_on_line

