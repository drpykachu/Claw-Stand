import time
import numpy as np
from st3215 import ST3215

# ================= Functions ===================


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

def circle(r, h, degree):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(np.deg2rad(degree))
    x = r * np.cos(np.deg2rad(degree))
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
    linearspace = np.linspace(0, 1, int(point_lower/2))
    expospace = linearspace**0.5
    expospace = np.concatenate((np.flip(expospace),-expospace))
    bot_path_circ = circle(R_tar, H_tar, -360/num_fingers_p1/2*expospace)    
    
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


