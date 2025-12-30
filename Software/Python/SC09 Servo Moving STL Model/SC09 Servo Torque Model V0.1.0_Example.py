import sys
import numpy as np
import tkinter as tk
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import AutoMinorLocator
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

matplotlib.use("TkAgg")

# ================= External Imports =================
from Claw_Functions import *  # Homebrew package

# ================= Parameters =======================

# Geometry (mm)
A = 33.55
B = 43.55
C = 43.55
T = 50

NUM_FINGERS = 5
OFFSET_R = 75
R_TARGET = 100
H_TARGETS = [130, 140, 150, 160]
H_TARGETS = [160, 150, 140, 130]

DELTA_Z = 20
POINTS = 100

# Masses (kg)
WEIGHT_BEARING = 13 / 1000
WEIGHT_MOTOR = 21 / 1000
BUFFER = 5 / 1000 # extra plastic, etc.

HOLDER_MOTOR_B = 17 / 1000 + BUFFER
HOLDER_MOTOR_C = 15 / 1000 + BUFFER
HOLDER_MOTOR_T = 8 / 1000 + BUFFER

# Motor / Spring
MOTOR_TORQUE_KGCM = 1.3
MOTOR_TORQUE = MOTOR_TORQUE_KGCM * 98.0665 / 1000

SPRING_K = 1.4 # from McMaster Carr
SPRING_K = (SPRING_K / 8.85075) / 270
G = 9.8
print(SPRING_K)
# Load
PLATE_LB = 0 /453.592
PLATE_LB = 0.5
PLATE_KG = (PLATE_LB / 2.20462) / 4

# ================= GUI ==============================

root = tk.Tk()
root.title("3D Finger Model - Degree Control")

fig, axs = plt.subplots(
    1, 3, sharex=True, sharey=True,
    figsize=(6, 3),
    constrained_layout=True
)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# ================= Helper Functions =================

def compute_operating_angles(master_path, load_mask):
    theta_a = np.zeros(POINTS)
    theta_b = np.zeros(POINTS)
    theta_c = np.zeros(POINTS)

    warned = False
    for i in range(POINTS):
        X, Y, Z = master_path[0, :, i]
        try:
            a, b, c = solve_thetas(Z, Y, X, A, B, C, T, OFFSET_R)[0]
            theta_a[i], theta_b[i], theta_c[i] = a, b, c
        except Exception:
            if not warned:
                print("WARNING: Some path points have no solution.")
                warned = True

    def valid_range(theta, mask):
        arr = theta * 180 / np.pi * mask
        arr = arr[arr != 0]
        return [np.min(arr), np.max(arr)]

    return [
        valid_range(theta_c, load_mask[2]),
        valid_range(theta_b, load_mask[1]),
        valid_range(theta_a, load_mask[0]),
    ]


def motor_loads(active):
    load = PLATE_KG if active else 0

    weights = np.array([
        [0, 0, HOLDER_MOTOR_T + load],
        [0, HOLDER_MOTOR_C + WEIGHT_MOTOR + WEIGHT_BEARING, HOLDER_MOTOR_T + load],
        [
            HOLDER_MOTOR_B + WEIGHT_MOTOR + WEIGHT_BEARING,
            HOLDER_MOTOR_C + WEIGHT_MOTOR + WEIGHT_BEARING,
            HOLDER_MOTOR_T + load,
        ],
    ])

    lengths = np.array([
        [0, 0, T / 1000],
        [0, B / 1000, B / 1000], #since T is worst case at like 90, i kept it as B length
        [C / 1000, (B+C) / 1000, (B+C+T)/ 1000],
    ])

    return weights, lengths


# ================= Update Function ==================

def update(idx):
    H_tar = H_TARGETS[idx]
    slider.config(label=f"Target Height: {H_tar} mm")

    fingertip_path = pathing(
        POINTS, DELTA_Z, NUM_FINGERS, R_TARGET, H_tar
    )

    master_path = np.ones((NUM_FINGERS, 3, POINTS))
    load_len = int(POINTS / NUM_FINGERS * (NUM_FINGERS - 1))

    load_mask = np.concatenate(
        (
            np.ones((NUM_FINGERS, 1, load_len)),
            np.zeros((NUM_FINGERS, 1, POINTS - load_len)),
        ),
        axis=2,
    )

    for i in range(NUM_FINGERS):
        shift = int(POINTS / NUM_FINGERS * i)
        master_path[i] = np.roll(fingertip_path, shift, axis=1)

    minmax = compute_operating_angles(master_path, load_mask)

    x = np.linspace(0, 180, 361)
    titles = ["Top Motor (C)", "Middle Motor (B)", "Bottom Motor (A)"]

    for i, ax in enumerate(axs):
        ax.cla()

        wt, st = [], []

        for theta in x:
            active = minmax[i][0] <= theta <= minmax[i][1]
            weights, lengths = motor_loads(active)

            moments = np.sum(weights * lengths, axis=1)
            wt.append(moments[i] * G * np.cos(np.deg2rad(theta)) * 1000)
            st.append(SPRING_K * theta * 1000)

        ax.plot(x, np.ones_like(x) * MOTOR_TORQUE * 1000, "k", label="Motor")
        ax.plot(x, -np.ones_like(x) * MOTOR_TORQUE * 1000, "k")
        ax.plot(x, wt, "b", alpha=0.3, label="Weight")
        ax.plot(x, st, "r", alpha=0.3, label="Spring")
        ax.plot(x, np.array(wt) + np.array(st), "m", label="Total")
        ax.axhline(0, color="k", linestyle="dotted", alpha=0.5)

        ax.add_patch(
            patches.Rectangle(
                (minmax[i][0], 0),
                minmax[i][1] - minmax[i][0],
                MOTOR_TORQUE * 1000,
                alpha=0.5,
                color="tab:orange",
                label = 'Load',
            )
        )

        ax.text(0.05, 0.05, r'%.2f lb Load' % PLATE_LB, transform=ax.transAxes, fontsize=9)
        ax.text(0.05, 0.9, titles[i], transform=ax.transAxes, fontsize=9)
        ax.set_xlim(0, 180)
        ax.set_xticks(range(0,136,45))
        ax.set_ylim(-200, 200)
        ax.xaxis.set_minor_locator(AutoMinorLocator(3))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))
        ax.tick_params(axis='both',
                       which='both',
                       direction='in',)    

        ax.set_xlabel(r"Theta / $\theta$")
        
        

    axs[0].set_ylabel("Torque / mN·m")
    
    axs[0].legend(loc = 4,
                  ncol = 1,
                  columnspacing=0.5,
                  handletextpad=0.2,
                  fontsize = 8.5,
                  handlelength = 1,
                  facecolor = 'w',
                  framealpha = 1,
                  fancybox = False)
    
    canvas.draw_idle()

    
# ================= Slider ===========================

slider = tk.Scale(
    root,
    from_=0,
    to=len(H_TARGETS) - 1,
    orient="horizontal",
    length=400,
    label="Target Height",
    command=lambda v: update(int(v)),
)

# slider.pack(fill=tk.X, padx=10, pady=10)

update(0)
root.mainloop()
