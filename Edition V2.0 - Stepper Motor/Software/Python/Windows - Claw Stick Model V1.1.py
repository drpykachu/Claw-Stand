import sys
import matplotlib
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

matplotlib.use("TkAgg")


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

#         print('ta: %.0f   tb: %.0f   td: %.0f     '% (np.rad2deg(theta_a), np.rad2deg(tb),np.rad2deg(td)))

        # ensures only one solution
        tc = td - tb

        if tc > 0:
            solutions.append((theta_a, tb, td))
            
    return solutions

def rotate(origin, point, degree):
    """Rotate a 2D point (x,y) around origin by angle (radians)."""
    ox, oy = origin
    px, py = point
    qx = ox + np.cos(np.deg2rad(degree)) * (px - ox) - np.sin(np.deg2rad(degree)) * (py - oy)
    qy = oy + np.sin(np.deg2rad(degree)) * (px - ox) + np.cos(np.deg2rad(degree)) * (py - oy)
    return qx, qy


def plotter(theta_a,theta_b,theta_c, Offset_R):
    """Return 3D coordinates of finger joints for a given circle degree."""
    theta_d = theta_b - theta_c
    
    # bottom - cyan
    x0 = 0 
    y0 = 0     
    x0 = 0 + Offset_R
    y0 = 0
    z0 = 0 

    # Black dot
    x1 = x0 + 0
    y1 = y0 + 0
    z1 = z0 + A 

    # Pink dot
    x2 = x1 + 0
    y2 = y1 + B*np.cos(theta_a)
    z2 = z1 + B*np.sin(theta_a)
#     x2, y2 = rotate((x1, y1), (x2,y2), Offset_theta)

    # Green dot
    x3 = x2 + C*np.cos(theta_b)
    y3 = y2 + C*np.sin(theta_b)*np.cos(theta_a)
    z3 = z2 + C*np.sin(theta_b)*np.sin(theta_a)
#     x3, y3 = rotate((x2, y2), (x3,y3), -Offset_theta)
    
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

# ================= Parameters ===================

A = 0 # length of motor A to pivot point
B = 15 # length of motor B to pivot point
C = 15  # length of motor C to pivot point
T = 20  # length of final digit T

# Xtar = 15  # Target point X
# Ytar = 10  # Target point Y
# Ztar = 30 # Target point Z


num_fingers = 5   # Number of fingers  
Offset_R    = 10  # Extension from origin
Offset_theta_master = np.linspace(360,0,num_fingers+1)[0:num_fingers]
print(Offset_theta_master)

R_tar = 35
H_tar = 35


# ================= Tkinter + Matplotlib Setup ===================

root = tk.Tk()
root.title("3D Finger Model - Degree Control")
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(str(int(screen_width/2))+"x"+str(screen_height-80)+"+0+0")

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection="3d")


lims = 75
ax.set_xlim([-lims, lims])
ax.set_ylim([-lims, lims])
ax.set_zlim([-lims/10, lims])

# ax.view_init(elev=90, azim=0) # X-Y plane initial view
ax.view_init(elev=0, azim=0) # Y-Z plane initial view
# ax.view_init(elev=0, azim=-90) # X-Z plane initial view
# ax.view_init(elev=15, azim=-45) # Trimetric inital view

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ================= Slider Callback ===================

def update(val):
    deg = np.deg2rad(val)
    ax.cla()
    
    # Sets axis labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    # Plots red circular trajectory
    circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, 361))
    ax.plot(circle_tar[0], circle_tar[1], circle_tar[2], 'r--', linewidth=1.5, label="Target Path")    

    # Plots blue base circle
    base_circle = circle(Offset_R, 0, np.linspace(0,360,101))
    ax.plot(base_circle[0], base_circle[1], base_circle[2], '--', c = 'b', lw = 2,alpha = 0.4)
    
    # Plots directional rose
    ax.plot([-lims, lims], [0, 0], [0, 0], '--', c = 'k', lw = 2,alpha = 0.2)
    ax.plot([0, 0], [-lims, lims], [0, 0], '--', c = 'k', lw = 2,alpha = 0.2)
    ax.plot([0, 0], [0, 0], [-lims/10, lims], '--', c = 'k', lw = 2,alpha = 0.2)

    
    for k in range(num_fingers):
        Xtar, Ytar, Ztar  = circle_tar[0][int(val)], circle_tar[1][int(val)], circle_tar[2]
        
        
        Offset_theta = Offset_theta_master[k]
        Xtar_new, Ytar_new = rotate((0,0), (Xtar, Ytar),Offset_theta)
        Ztar_new = Ztar
            
        # Solves for theta to reconstruct the model
        theta_a,theta_b,theta_c = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T,Offset_R)[0] 
        coords_unr = plotter(theta_a,theta_b,theta_c,Offset_R)
        
        coords = 0*coords_unr

        for v in range(np.shape(coords_unr)[1]):
            xold = coords_unr[0, v]
            yold = coords_unr[1, v]
            xnew, ynew = rotate((0,0), (xold,yold), Offset_theta)
            coords[0,v] = xnew
            coords[1,v] = ynew
            coords[2,v] = coords_unr[2, v]

        # Plots finger ball and stick model
        ax.plot(coords[0], coords[1], coords[2], 'black', linewidth=2)
        ax.plot([coords[0, 0]], [coords[1, 0]], [coords[2, 0]], 'oc', markersize=8)
        ax.plot([coords[0, 1]], [coords[1, 1]], [coords[2, 1]], 'ok', markersize=8)
        ax.plot([coords[0, 2]], [coords[1, 2]], [coords[2, 2]], 'om', markersize=8)
        ax.plot([coords[0, 3]], [coords[1, 3]], [coords[2, 3]], 'og', markersize=8)
        ax.plot([coords[0, 4]], [coords[1, 4]], [coords[2, 4]], 'or', markersize=8)
        

        X_tar_text,Y_tar_text = rotate((0,0), ((R_tar+7.5)*np.cos(np.deg2rad(Offset_theta+val)), (R_tar+7.5)*np.sin(np.deg2rad(Offset_theta+val))),Offset_theta)
        ax.text(X_tar_text, Y_tar_text, Ztar_new, str(k+1), color='r')

        ax.plot([Xtar_new], [Ytar_new], [Ztar_new], '.b', markersize=8,zorder = 100)
    
    canvas.draw_idle()
    
# ================= Slider Widget ===================
deg = 45
slider = tk.Scale(root, from_=-deg, to=deg, orient="horizontal", length=400,
                  label="Degree", command=lambda v: update(float(v)))
slider.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)

update(0)
root.mainloop()
