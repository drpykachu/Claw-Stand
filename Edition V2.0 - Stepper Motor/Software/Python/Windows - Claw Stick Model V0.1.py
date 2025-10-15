import numpy as np
import matplotlib
matplotlib.use("TkAgg")   # Force Tkinter backend
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import math


# ================= Functions ===================

def angle_finder(x,y,z):
    theta_b = np.arctan(z/x)
    c1 = (z - E*np.sin(theta_b))/M
    c2 = (y-P)/M
    A = 2*T*c1/M
    B = -2*T*c2/M
    phi = np.arcsin(((T/M)**2 + c1**2 + c2**2 - 1)/((A**2 + B**2)**0.5)) - np.arctan2(B,A)
    theta_m = np.arcsin(c1 - T*np.sin(phi)/M)
    return (theta_b,theta_m,phi)

def circle(r,h,degree):
    z = h
    y = r*np.sin(degree)
    x = r*np.cos(degree)
    return (x,y,z)

def rotate(origin, point, angle):
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def plotter(degree,offset_R,offset_angle):
    flipper = 0
    coordinate_track = circle(r-offset_R,h,degree)
    x_cir,y_cir,z_cir = coordinate_track
    if x_cir < 0:
        x_cir = -x_cir
        flipper = 1
    theta_b,theta_m,theta_phi = angle_finder(x_cir,y_cir,z_cir)
    theta_t = theta_phi + theta_m

    zm = M*np.sin(theta_m)+E*np.sin(theta_b)
    xm = (zm)/np.tan(theta_b)

    x_dot_1,y_dot_1,z_dot_1 = 0,0,0
    x_dot_2,y_dot_2,z_dot_2 = E*np.cos(theta_b),0,E*np.sin(theta_b)
    x_dot_3,y_dot_3,z_dot_3 = x_dot_2,P,z_dot_2
    x_dot_4,y_dot_4,z_dot_4 = xm,P + M*np.cos(theta_m),zm
    x_dot_5 = x_cir
    y_dot_5 = y_dot_4 - T*np.cos(theta_t-theta_m)
    z_dot_5 = z_dot_4 + T*np.sin(theta_t-theta_m)

    if flipper == 1:
        x_dot_1 = -x_dot_1
        x_dot_2 = -x_dot_2
        x_dot_3 = -x_dot_3
        x_dot_4 = -x_dot_4
        x_dot_5 = -x_dot_5

    # Apply offset to y only (like your script)
    y_dot_1 += offset_R
    y_dot_2 += offset_R
    y_dot_3 += offset_R
    y_dot_4 += offset_R
    y_dot_5 += offset_R

    return np.array([ [x_dot_1,x_dot_2,x_dot_3,x_dot_4,x_dot_5],
                      [y_dot_1,y_dot_2,y_dot_3,y_dot_4,y_dot_5],
                      [z_dot_1,z_dot_2,z_dot_3,z_dot_4,z_dot_5] ])


# ================= Parameters ===================

T = 105
M = 124
E = 41
P = 82
r = 150
h = 200
num_fingers = 5
offset_R = 50

offset_angle = np.deg2rad(360/num_fingers)


# ================= Tkinter + Matplotlib Setup ===================

root = tk.Tk()
root.title("3D Finger Model - Degree Control")

fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim([-250,250])
ax.set_ylim([-250,250])
ax.set_zlim([-250,250])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.view_init(elev=8, azim=-53)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)


# ================= Slider Callback ===================

def update(val):
    deg = np.deg2rad(val)
    ax.cla()  # clear axes
    
    # redraw axes limits
    ax.set_xlim([-250,250])
    ax.set_ylim([-250,250])
    ax.set_zlim([-250,250])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=8, azim=-53)
    
    # compute for each finger
    origin = (0,0)
    rotate_angle = np.linspace(0,2*np.pi,num_fingers,endpoint=False)
    for k in range(num_fingers):
        coords = plotter(deg,offset_R,offset_angle)
        # rotate
        for p in range(coords.shape[1]):
            coords[0,p],coords[1,p] = rotate(origin,(coords[0,p],coords[1,p]),rotate_angle[k])
        ax.plot(coords[0],coords[1],coords[2],'gray')
        ax.plot(coords[0][-1:],coords[1][-1:],coords[2][-1:],'or')
    canvas.draw_idle()


# ================= Slider Widget ===================

slider = tk.Scale(root, from_=0, to=360, orient="horizontal", length=400,
                  label="Degree", command=lambda v: update(float(v)))
slider.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)


update(60)  # initialize at 60 degrees

root.mainloop()

