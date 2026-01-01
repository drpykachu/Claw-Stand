import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import math


# ================= Functions ===================

def solve_thetas(Zp, Yp, Xp, A, B, C, T):
    if Yp < 0.0005:
        Yp = 0.0005        
        
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
        solutions.append((theta_a, tb, td))
    return solutions


def plotter(theta_a,theta_b,theta_c):

    theta_d = theta_b - theta_c
    
    # bottom - no dot
    x0 = 0
    y0 = 0
    z0 = 0 

    # Black dot
    x1 = x0 + 0
    y1 = y0 + 0
    z1 = z0 + A 

    # Pink dot
    x2 = x1 + 0
    y2 = x1 + B*np.cos(theta_a)
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


# ================= Parameters ===================

A = 10 # length of motor A to pivot point
B = 10 # length of motor B to pivot point
C = 8  # length of motor C to pivot point
T = 8  # length of final digit T

Xp = 0
Yp = 5
Zp = 25

theta_a,theta_b,theta_c = solve_thetas(Zp, Yp, Xp, A, B, C, T)[1] 
coords = plotter(theta_a,theta_b,theta_c)


# ================= Tkinter + Matplotlib Setup ===================

# root = tk.Tk()
# root.title("3D Finger Model - Degree Control")

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection="3d")

lims = 40
ax.set_xlim([-lims, lims])
ax.set_ylim([-lims, lims])
ax.set_zlim([-lims, lims])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.view_init(elev=8, azim=-53)


# rotate finger about z

ax.plot(coords[0], coords[1], coords[2], 'black', linewidth=2)
ax.plot([coords[0, 0]], [coords[1, 0]], [coords[2, 0]], 'oc', markersize=8)
ax.plot([coords[0, 1]], [coords[1, 1]], [coords[2, 1]], 'ok', markersize=8)
ax.plot([coords[0, 2]], [coords[1, 2]], [coords[2, 2]], 'om', markersize=8)
ax.plot([coords[0, 3]], [coords[1, 3]], [coords[2, 3]], 'og', markersize=8)
ax.plot([coords[0, 4]], [coords[1, 4]], [coords[2, 4]], 'or', markersize=8)

ax.plot([Xp], [Yp], [Zp], '.b', markersize=8)


plt.show()
