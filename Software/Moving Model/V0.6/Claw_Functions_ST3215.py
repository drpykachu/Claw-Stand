import sys
import vtk
import pyvista as pv
from pyvistaqt import QtInteractor
import trimesh
import itertools
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
import matplotlib.colors as mcolors
import matplotlib.pyplot as plt
import pyautogui  # to automatically detect screen size

# ================= Functions ===================

def bounds_overlap(mesh1, mesh2):
    b1 = mesh1.bounds
    b2 = mesh2.bounds

    return (
        b1[1] >= b2[0] and b2[1] >= b1[0] and
        b1[3] >= b2[2] and b2[3] >= b1[2] and
        b1[5] >= b2[4] and b2[5] >= b1[4]
    )


def meshes_intersect(mesh1, mesh2, tol=0.0):
    collision = vtk.vtkCollisionDetectionFilter()
    collision.SetInputData(0, mesh1)
    collision.SetInputData(1, mesh2)

    t1 = vtk.vtkTransform()
    t1.Identity()
    t2 = vtk.vtkTransform()
    t2.Identity()

    collision.SetTransform(0, t1)
    collision.SetTransform(1, t2)

    collision.SetCollisionModeToAllContacts()
    collision.SetBoxTolerance(tol)
    collision.SetCellTolerance(tol)

    collision.Update()

    return collision.GetNumberOfContacts() > 0


def check_no_intersections(pyvista_poly_dict):
    keys = list(pyvista_poly_dict.keys())

    for k1, k2 in itertools.combinations(keys, 2):
        m1 = pyvista_poly_dict[k1]
        m2 = pyvista_poly_dict[k2]

        # Fast reject
        if not bounds_overlap(m1, m2):
            continue

        # Accurate test
        if meshes_intersect(m1, m2):
            return False, (k1, k2)

    return True, None


def patterns(kind, num_fingers,points,fingertip_path):
    master_path = np.ones((num_fingers,3,points))
    if kind == 'Roll':
        # ROLLING PATTERN SHIFT - goes from tip to tip
        for i in range(num_fingers):
            """ Loop to adjust the finger path for each finger, offset by the points/fingers for even spacing."""
            shifter = int(points / (num_fingers) * i)
            shifted = np.roll(fingertip_path, shift=shifter, axis=1)
            master_path[i] = shifted
            
            
    if kind == 'Star':
        # STAR PATTERN SHIFT (e.g. 1→3→5→2→4 for 5 fingers)
        step = 2                      # star step (works for odd num_fingers)
        spacing = int(points / num_fingers)
        order = []
        current = 0
        while current not in order:
            order.append(current)
            current = (current + step) % num_fingers

        for shift_idx, finger_idx in enumerate(order):
            shifter = spacing * shift_idx
            shifted = np.roll(fingertip_path, shift=shifter, axis=1)
            master_path[finger_idx] = shifted
            
    return master_path

def ang2bit(angle_deg):
    pos = int(4096 * angle_deg / 360)
    return pos

def solve_thetas(Zp, Yp, Xp, A, A_x, B, C, T, Offset_R):
    """Returns the angles (in radians) of motors to obtain a point in space (Xp, Yp, Zp)"""
    Xp = Xp - Offset_R - A_x
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


def point_coords(theta_a,theta_b,theta_c, Offset_R, A, A_x, B, C, T):
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

    # Magenta dot
    x2 = x1 + A_x
    y2 = y1
    z2 = z1 

    # Pink dot
    x3 = x2
    y3 = y2 + B*np.cos(theta_a)
    z3 = z2 + B*np.sin(theta_a)

    # Green dot
    x4 = x3 + C*np.cos(theta_b)
    y4 = y3 + C*np.sin(theta_b)*np.cos(theta_a)
    z4 = z3 + C*np.sin(theta_b)*np.sin(theta_a)
    
    # Red dot
    x5 = x4 + T*np.cos(theta_c)
    y5 = y4 + T*np.sin(theta_c)*np.cos(theta_a)
    z5 = z4 + T*np.sin(theta_c)*np.sin(theta_a)

    return np.array([
        [x0, x1, x2, x3, x4, x5],
        [y0, y1, y2, y3, y4, y5],
        [z0, z1, z2, z3, z4, z5]
    ])


def circle(r, h, degree):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(np.deg2rad(degree))
    x = r * np.cos(np.deg2rad(degree))
    return [x, y, z]

def circle_origin(r, h, degree, origin = [0,0]):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(np.deg2rad(degree)) + origin[1]
    x = r * np.cos(np.deg2rad(degree)) + origin[0]
    return [x, y, z]

def pathing(points, delta_Z,num_fingers,R_tar, H_tar):
    """Builds the path for the fingertip travel."""
    exponent = 4
    num_fingers_p1 = num_fingers + 1
    
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
    bot_path_circ = circle(R_tar, H_tar, np.linspace(-360/num_fingers_p1/2, 360/num_fingers_p1/2, point_lower))     # for linear spacing


    bot_func = delta_Z*np.flip(bot_path_circ[1])**exponent / np.max(bot_path_circ[1]**exponent)  + (bot_path_circ[2] - delta_Z)
    bot_path = [np.flip(bot_path_circ[0]),np.flip(bot_path_circ[1]),bot_func]    
     
    total_path = np.concatenate((top_path, bot_path), axis=1)    
    
    return total_path

def master_pathing_fix(points, delta_Z,num_fingers,R_tar, H_tar, error_distance, correction_angle, Offset_theta_master, val_anim):
    """Builds path to fix the correct position. It contains the combination of two pathing - the correct and fixed path."""
    
    #### 1. Generates first sequence - normal top path and bottom path.
    exponent = 4
    num_fingers_p1 = num_fingers + 1

    # distributing points
    point_upper = int(points/num_fingers*(num_fingers-1))
    point_lower = int(points/num_fingers)

    # building top path
    top_path = circle(R_tar, H_tar, np.linspace(-360/num_fingers_p1/2, 360/num_fingers_p1/2, point_upper))
    top_path[2] = np.ones(len(top_path[0]))*top_path[2]
    
    # building bottom path
    bot_path_circ = circle(R_tar, H_tar, np.linspace(-360/num_fingers_p1/2, 360/num_fingers_p1/2, point_lower))     # for linear spacing
    bot_func = delta_Z*np.flip(bot_path_circ[1])**exponent / np.max(bot_path_circ[1]**exponent)  + (bot_path_circ[2] - delta_Z)
    bot_path = [np.flip(bot_path_circ[0]),np.flip(bot_path_circ[1]),bot_func]    
     
    total_path = np.concatenate((top_path, bot_path), axis=1)    
    
    
    #### 2. Then, it does the rolling of the path to offset per finger
    master_path = np.ones((num_fingers,3,points))
    for i in range(num_fingers):
        """ Loop to adjust the finger path for each finger, offset by the points/fingers for even spacing."""
        shifter = int(points / (num_fingers) * i)
        shifted = np.roll(total_path, shift=shifter, axis=1)
        master_path[i] = shifted


    #### 3. Now, lets find the index of where each finger starts its return.
    bot_index_finder = []
    for k in range(0,num_fingers):
        flag = False
        for q in range(0,len(master_path[k][2])):
            if master_path[k][2][q] == H_tar:
                flag = True
            if flag == True and master_path[k][2][q] < H_tar:
                bot_index_finder.append(q)
                break
    
    #### 4. Now, lets find the index of where each finger finds its way back from bottom to top            
    top_index_finder = []
    for k in range(0,num_fingers):
        flag = False
        for q in range(0,len(master_path[k][2])):
            if master_path[k][2][q] < H_tar:
                flag = True
            if flag == True and master_path[k][2][q] == H_tar:
                top_index_finder.append(q)
                break
            
    #### 5. Now, for all points leading up to the bottom index, let's employ the fix for the new top path
    for k in range(0,num_fingers):
        for q in range(0,top_index_finder[k]):        
            Xtar_fix = master_path[k][0][q]
            Ytar_fix = master_path[k][1][q]
        
            Offset_theta = Offset_theta_master[k]
            Xtar_fix, Ytar_fix = rotate_point([0, 0], [Xtar_fix, Ytar_fix], Offset_theta )                             
            Xtar_fix = Xtar_fix + error_distance*np.cos(np.deg2rad(correction_angle))
            Ytar_fix = Ytar_fix + error_distance*np.sin(np.deg2rad(correction_angle))
            Xtar_fix, Ytar_fix = rotate_point([0, 0], [Xtar_fix, Ytar_fix], -Offset_theta )
            
            master_path[k][0][q] = Xtar_fix
            master_path[k][1][q] = Ytar_fix
            
    
    #### 6. Now, lets correct the bottom path between the indexes of bottom_index to top_index
    for k in range(0,num_fingers):            
        Xtar_fix_start = master_path[k][0][bot_index_finder[k]]
        Ytar_fix_start = master_path[k][1][bot_index_finder[k]]
        
        Xtar_fix_end = master_path[k][0][top_index_finder[k]]
        Ytar_fix_end = master_path[k][1][top_index_finder[k]]
        
        new_path_x = np.linspace(Xtar_fix_start,Xtar_fix_end, abs(top_index_finder[k] - bot_index_finder[k]))
        new_path_y = np.linspace(Ytar_fix_start,Ytar_fix_end, abs(top_index_finder[k] - bot_index_finder[k]))
        master_path[k][0][min(bot_index_finder[k],top_index_finder[k]):max(bot_index_finder[k],top_index_finder[k])] = new_path_x
        master_path[k][1][min(bot_index_finder[k],top_index_finder[k]):max(bot_index_finder[k],top_index_finder[k])] = new_path_y
            
    #### 7. Let's set the first data point equal to the second data point so its a next step, not a repeated step 
    for k in range(0,num_fingers):
        master_path[k][0][0] = master_path[k][0][1]
        master_path[k][1][0] = master_path[k][1][1]
        master_path[k][2][0] = master_path[k][2][1]
        
    return master_path
        
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

