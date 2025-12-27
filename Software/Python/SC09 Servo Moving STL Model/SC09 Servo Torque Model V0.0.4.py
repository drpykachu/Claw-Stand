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

# System
NUM_FINGERS = 5
OFFSET_R = 75
R_TARGET = 100
H_TARGETS = [130, 135, 140, 145, 150, 155, 160]

DELTA_Z = 20
POINTS = 100

# Masses (kg)
WEIGHT_BEARING = 10 / 1000
WEIGHT_MOTOR = 21 / 1000
BUFFER = 5 / 1000

HOLDER_MOTOR_B = 17 / 1000 + BUFFER
HOLDER_MOTOR_C = 15 / 1000 + BUFFER
HOLDER_MOTOR_T = 8 / 1000 + BUFFER

# Motor / Spring
MOTOR_TORQUE_KGCM = 1.3
KGCM_TO_NM = 98.0665 / 1000
MOTOR_TORQUE = MOTOR_TORQUE_KGCM * KGCM_TO_NM

SPRING_CONSTANT_INLBF = 1.4
INLBF_TO_NM = 1 / 8.85075
THETA_DEFLECTION = 270
SPRING_K = (SPRING_CONSTANT_INLBF * INLBF_TO_NM) / THETA_DEFLECTION

G = 9.8

# Load
PLATE_LB = 2
PLATE_KG = (PLATE_LB / 2.20462) / 4  # shared by 4 fingers

# ================= GUI ==============================

root = tk.Tk()
root.title("3D Finger Model - Degree Control")

fig, axs = plt.subplots(
    3, 1, sharex=True, sharey=True,
    figsize=(3, 6),
    constrained_layout=True
)

canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# ================= Helper Functions =================


def compute_operating_angles(master_path, load_mask):
    """Find min/max operating angles for motors A/B/C."""
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


def motor_loads(theta, active):
    """Return weights and lengths per motor."""
    load = PLATE_KG if active else 0

    weights = [
        [0, 0, HOLDER_MOTOR_T + load],
        [0, HOLDER_MOTOR_C + WEIGHT_MOTOR + WEIGHT_BEARING, HOLDER_MOTOR_T + load],
        [
            HOLDER_MOTOR_B + WEIGHT_MOTOR + WEIGHT_BEARING,
            HOLDER_MOTOR_C + WEIGHT_MOTOR + WEIGHT_BEARING,
            HOLDER_MOTOR_T + load,
        ],
    ]

    lengths = [
        [0, 0, T / 1000],
        [0, B / 1000, T / 1000],
        [C / 1000, B / 1000, T / 1000],
    ]

    return np.array(weights), np.array(lengths)


# ================= Update Function ==================

def update(idx):
    H_tar = H_TARGETS[idx]

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

    titles = [
        "i.) Top Motor (C)",
        "ii.) Middle Motor (B)",
        "iii.) Bottom Motor (A)",
    ]

    for i, ax in enumerate(axs):
        ax.cla()

        wt, st = [], []

        for theta in x:
            active = minmax[i][0] <= theta <= minmax[i][1]
            weights, lengths = motor_loads(theta, active)

            moments = np.sum(weights * lengths, axis=1)
            torque_weight = moments[i] * G * np.cos(np.deg2rad(theta))
            torque_spring = SPRING_K * theta

            wt.append(torque_weight * 1000)
            st.append(torque_spring * 1000)

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
            )
        )

        ax.text(
            0.025, 0.925, titles[i],
            transform=ax.transAxes,
            fontsize=9
        )

        ax.set_xlim(0, 180)
        ax.set_ylim(-200, 200)
        ax.xaxis.set_minor_locator(AutoMinorLocator(3))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))
        ax.tick_params(direction="in")

    axs[2].set_xlabel(r"Theta / $\theta$")
    axs[1].set_ylabel("Torque / mN·m")

    canvas.draw_idle()


# ================= Slider ===========================

slider = tk.Scale(
    root,
    from_=0,
    to=len(H_TARGETS) - 1,
    orient="horizontal",
    length=400,
    label="Height Index",
    command=lambda v: update(int(v)),
)

slider.pack(fill=tk.X, padx=10, pady=10)

update(0)
root.mainloop()
