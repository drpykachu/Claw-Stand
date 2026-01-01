import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import math


# ================= Functions ===================

def circle(r, h, degree):
    """Return (x, y, z) coordinates of a circular trajectory."""
    z = h
    y = r * np.sin(degree)
    x = r * np.cos(degree)
    return x, y, z


def rotate(origin, point, angle):
    """Rotate a 2D point (x,y) around origin by angle (radians)."""
    ox, oy = origin
    px, py = point
    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy


def plotter(degree, offset_R, offset_angle):
    """Return 3D coordinates of finger joints for a given circle degree."""
    # Target fingertip on circular path
    x_target, y_target, z_target = circle(r - offset_R, h, degree)
    y_target += offset_R  # add finger offset

    # Vector from base to target
    dx = x_target
    dy = y_target - offset_R
    dz = z_target - offset_Z

    # Projected planar distance (xz-plane)
    Lxz = np.hypot(dx, dz)
    L = np.hypot(Lxz, dy)

    # Law of cosines
    if L > (M + T):
        L = M + T  # clamp if unreachable

    theta_b = np.arctan2(dz, dx)  # base tilt
    theta_m = np.arccos((M**2 + L**2 - T**2) / (2 * M * L))
    theta_t = np.pi - np.arccos((M**2 + T**2 - L**2) / (2 * M * T)) + theta_m

    # Joint positions (forward kinematics)
    x0, y0, z0 = 0, offset_R, offset_Z
    x1 = E * np.cos(theta_b)
    y1 = offset_R
    z1 = E * np.sin(theta_b)

    x2 = x1 + M * np.cos(theta_m) * np.cos(theta_b)
    y2 = y1 + M * np.sin(theta_m)
    z2 = z1 + M * np.cos(theta_m) * np.sin(theta_b)

    # Fingertip = exact circle point
    x3, y3, z3 = x_target, y_target, z_target

    return np.array([
        [x0, x1, x2, x3],
        [y0, y1, y2, y3],
        [z0, z1, z2, z3]
    ])


# ================= Parameters ===================

T = 100  # top length
M = 100  # middle length
P = 0
E = 0

r = 150  # circle radius
h = 200  # circle height
num_fingers = 1

offset_R = 50
offset_Z = 50
offset_angle = np.deg2rad(360 / num_fingers)


# ================= Tkinter + Matplotlib Setup ===================

root = tk.Tk()
root.title("3D Finger Model - Degree Control")

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection="3d")

lims = 250
ax.set_xlim([-lims, lims])
ax.set_ylim([-lims, lims])
ax.set_zlim([-lims, lims])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.view_init(elev=8, azim=-53)

yy, xx = np.meshgrid(range(-lims, lims), range(-lims, lims))
zz = yy * 0
ax.plot_surface(xx, yy, zz, color="gray", alpha=0.5)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)


# ================= Slider Callback ===================

def update(val):
    deg = np.deg2rad(val)
    ax.cla()

    # Red circular trajectory
    dvals = np.linspace(0, 2 * np.pi, 200)
    path_x, path_y, path_z = circle(r, h, dvals)
    ax.plot(path_x, path_y, path_z, 'r--', linewidth=1.5, label="Target Path")

    # Reset axes
    ax.set_xlim([-lims, lims])
    ax.set_ylim([-lims, lims])
    ax.set_zlim([-lims, lims])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=50, azim=-45)
    ax.plot_surface(xx, yy, zz, color="gray", alpha=0.5)

    origin = (0, 0)
    rotate_angle = np.linspace(0, 2 * np.pi, num_fingers, endpoint=False)

    for k in range(num_fingers):
        coords = plotter(deg, offset_R, offset_angle)
        # rotate finger about z
        for p in range(coords.shape[1]):
            coords[0, p], coords[1, p] = rotate(origin, (coords[0, p], coords[1, p]), rotate_angle[k])
        ax.plot(coords[0], coords[1], coords[2], 'black', linewidth=2)
        ax.plot([coords[0, -1]], [coords[1, -1]], [coords[2, -1]], 'or', markersize=8)

    canvas.draw_idle()


# ================= Slider Widget ===================

slider = tk.Scale(root, from_=0, to=360, orient="horizontal", length=400,
                  label="Degree", command=lambda v: update(float(v)))
slider.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)

update(0)
root.mainloop()
