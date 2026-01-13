import sys
import matplotlib
import numpy as np
import tkinter as tk
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patheffects as path_effects
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

def pathing(points, delta_Z,num_fingers):
    """Builds the path for the fingertip travel. Top pathis set to portion of circle, bottom path set to an exponential function"""
    exponent = 6
    num_fingers_p1 = num_fingers+1
    
    class PointCount(Exception):
        """Checks if the amount of points is even."""
        pass

    if points % 2 != 0:
        raise PointCount("Number of points must be even.")

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
    """Rotate a 2D point (x,y) around origin by angle (radians)."""
    ox, oy = origin
    px, py = point
    qx = ox + np.cos(np.deg2rad(degree)) * (px - ox) - np.sin(np.deg2rad(degree)) * (py - oy)
    qy = oy + np.sin(np.deg2rad(degree)) * (px - ox) + np.cos(np.deg2rad(degree)) * (py - oy)
    return qx, qy


def rotate_vector(vec, degree, origin=(0, 0)):
    """    Rotate all (x, y) points in a 2×N or 3×N vector around the given origin.    """
    ox, oy = origin
    theta = np.deg2rad(degree)

    # Rotation matrix
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])

    # Handle both (2, N) and (3, N)
    XY = vec[:2, :]
    XY_shifted = XY - np.array([[ox], [oy]])
    XY_rot = R @ XY_shifted + np.array([[ox], [oy]])

    if vec.shape[0] == 3:
        rotated = np.vstack((XY_rot, vec[2, :]))
    else:
        rotated = XY_rot

    return rotated

# ================= Parameters ===================

A = 10 # length of motor A to pivot point
B = 15 # length of motor B to pivot point
C = 15  # length of motor C to pivot point
T = 20  # length of final digit T

# Xtar = 15  # Target point X
# Ytar = 10  # Target point Y
# Ztar = 30 # Target point Z


num_fingers = 5   # Number of fingers  
Offset_R    = 20  # Extension from origin
Offset_theta_master = np.linspace(360,0,num_fingers+1)[0:num_fingers]

R_tar = 35
H_tar = 45

circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, 361))

############ Initializes path for each finger ############
points = 100 # points per path
delta_Z = 5

master_path = np.ones((num_fingers,3,points))
fingertip_path = pathing(points, delta_Z,num_fingers)

for i in range(num_fingers):
    shifter = int(points / (num_fingers) * i)
    shifted = np.roll(fingertip_path, shift=shifter, axis=1)
    master_path[i] = shifted

path_colors = ['tab:green','red','tab:orange','cyan','magenta']
joint_colors = ['#FFFFFF', '#BFBFBF', '#808080', '#404040', '#000000']

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

ax.view_init(elev=90, azim=0) # X-Y plane initial view
# ax.view_init(elev=0, azim=0) # Y-Z plane initial view
# ax.view_init(elev=0, azim=-90) # X-Z plane initial view
ax.view_init(elev=15, azim=-45) # Trimetric inital view

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# ================= Slider Callback ===================

def update(val):
    val = val - 1
    deg = np.deg2rad(val)
    ax.cla()
    
    # Sets axis labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    
    # Plots red circular trajectory
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
        Xtar, Ytar, Ztar = master_path[k, :, int(val)]
        
        
        Offset_theta = Offset_theta_master[k]
        Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar),Offset_theta)
        Ztar_new = Ztar
        
        
        tex_off = 10
        Xtar_text = Xtar_new + tex_off * np.cos(np.deg2rad(Offset_theta))
        Ytar_text = Ytar_new + tex_off * np.sin(np.deg2rad(Offset_theta))            

        # Solves for theta to reconstruct the model
        try:
            theta_a,theta_b,theta_c = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T,Offset_R)[0] 
            coords_unr = plotter(theta_a,theta_b,theta_c,Offset_R)
            
            coords = 0*coords_unr

            coords = rotate_vector(coords_unr, Offset_theta)

            # Plots finger ball and stick model
            ax.plot(coords[0], coords[1], coords[2], 'black', linewidth=2)
            ax.plot([coords[0, 0]], [coords[1, 0]], [coords[2, 0]], 'o', c = 'tab:blue', markersize=8)
            ax.plot([coords[0, 1]], [coords[1, 1]], [coords[2, 1]], 'o', c = joint_colors[1], markersize=8)
            ax.plot([coords[0, 2]], [coords[1, 2]], [coords[2, 2]], 'o', c = joint_colors[2], markersize=8)
            ax.plot([coords[0, 3]], [coords[1, 3]], [coords[2, 3]], 'o', c = joint_colors[3], markersize=8)
            if np.round(coords[2, 4],2) != H_tar:                
                ax.plot([coords[0, 4]], [coords[1, 4]], [coords[2, 4]], 'o', color = path_colors[k], markersize=8,zorder = 101)
            else:
                ax.plot([coords[0, 4]], [coords[1, 4]], [coords[2, 4]], 'o', color = path_colors[k], markersize=8,zorder = 99)
                
        except:
            1


        # plots fingertip path
        master_path_plot = rotate_vector(master_path[k], Offset_theta)
        ax.plot(master_path_plot[0], master_path_plot[1], master_path_plot[2], path_colors[k], markersize=8,zorder = 50)

        # plots fingertip location - and shows if contact or not
        ax.plot(Xtar_new, Ytar_new, Ztar_new, '.k', markersize=8,zorder = 100) 
              
        txt = ax.text(Xtar_text, Ytar_text, Ztar_new, str(k+1),color=path_colors[k], fontsize=12,zorder = 101)

        # Add a black outline
        txt.set_path_effects([path_effects.Stroke(linewidth=2, foreground='black'), path_effects.Normal()])

    canvas.draw_idle()
    
# ================= Slider Widget ===================
slider = tk.Scale(root, from_= 1, to=points, orient="horizontal", length=400,
                  label="Degree", command=lambda v: update(float(v)))
slider.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)

update(0)
root.mainloop()
