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
from Claw_Functions_ST3215 import *  # Homebrew package

# ================= Parameters =======================


# Digit Lengths
A = 45 # bottom digit + motor A height
B = 19.8 # lower-middle digit + motor B height

C = 75 # upper-middle middle 2 digit + motor B height
T = 65 # top digit height

A_x = 39.8

NUM_FINGERS = 5
Offset_R    = 60.075  # From origin (0,0,0)
R_TARGET = 80
H_tar    = 190  # Height of target circle path

DELTA_Z = 20
POINTS = 200

# Masses (kg)
WEIGHT_BEARING = 0 / 1000
WEIGHT_MOTOR = 70 / 1000
BUFFER = 15 / 1000 # extra plastic, etc.

HOLDER_MOTOR_B = 30 / 1000 + BUFFER
HOLDER_MOTOR_C = 30 / 1000 + BUFFER
HOLDER_MOTOR_T = 30 / 1000 + BUFFER

# Motor / Spring
MOTOR_TORQUE_KGCM = 30
MOTOR_TORQUE = MOTOR_TORQUE_KGCM * 98.0665 / 1000



deflect_angle = 270
extra_winds = 1
SPRING_K = 7.5 # from McMaster Carr 9271K581
Sl = 0.07*8.75
print(Sl)

deflect_angle = 270
extra_winds = 1
SPRING_K = 7.5 # from McMaster Carr 9271K581
Sl = 0.059*8.75
print(Sl)

# deflect_angle = 360
# extra_winds = 0
# SPRING_K = 9.2 # from McMaster Carr 9271K967
# Sl = 0.078*10.5
# print(Sl)

SPRING_K = (SPRING_K / 8.85075) / 270
G = 9.8

# Load
PLATE_LB = 10
PLATE_KG = (PLATE_LB / 2.20462) / 4



# ================= Helper Functions =================

def compute_operating_angles(master_path, load_mask):
    theta_a = np.zeros(POINTS)
    theta_b = np.zeros(POINTS)
    theta_c = np.zeros(POINTS)

    warned = False
    for i in range(POINTS):
        Xtar, Ytar, Ztar = master_path[0, :, i]
        try:
            a, b, c = solve_thetas(Ztar, Ytar, Xtar, A, A_x, B, C, T,Offset_R)[0]
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
        [0, B / 1000, (B+C) / 1000],
        [C / 1000, (B+C) / 1000, (B+C+T)/ 1000],
    ])

    return weights, lengths





fingertip_path = pathing(POINTS, DELTA_Z, NUM_FINGERS, R_TARGET, H_tar)

master_path = np.ones((NUM_FINGERS, 3, POINTS))
load_len = int(POINTS / NUM_FINGERS * (NUM_FINGERS - 1))

load_mask = np.concatenate(
    (   np.ones((NUM_FINGERS, 1, load_len)),
        np.zeros((NUM_FINGERS, 1, POINTS - load_len)),    ),
    axis=2,
)

for i in range(NUM_FINGERS):
    shift = int(POINTS / NUM_FINGERS * i)
    master_path[i] = np.roll(fingertip_path, shift, axis=1)

minmax = compute_operating_angles(master_path, load_mask)

x = np.linspace(-0, 180, 361)
titles = ["Top Motor (C)", "Middle Motor (B)", "Bottom Motor (A)"]



##################### Plotting

fig, ax = plt.subplots(
    1, 1, sharex=True, sharey=True,
    figsize=(6, 3),
    constrained_layout=True
)



wt, st = [], []
i = 2

theta_offfset  = 90

for theta in x:
    active = minmax[i][0] <= theta <= minmax[i][1]
    weights, lengths = motor_loads(active)

    moments = np.sum(weights * lengths, axis=1)
    wt.append(moments[i] * G * np.cos(np.deg2rad(theta)) * 1000)
    st.append(SPRING_K * (theta - ((deflect_angle+360*extra_winds) - theta_offfset) ) * 1000)

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

ax.text(0.05, 0.1, r'%.2f lb Load' % PLATE_LB, transform=ax.transAxes, fontsize=9)
ax.text(0.05, 0.9, titles[i], transform=ax.transAxes, fontsize=9)
ax.set_xlim(-0, 180)
ax.set_xticks(range(-0,136,45))
#         ax.set_ylim(-200, 200)
ax.xaxis.set_minor_locator(AutoMinorLocator(3))
ax.yaxis.set_minor_locator(AutoMinorLocator(2))
ax.tick_params(axis='both',
               which='both',
               direction='in',)    

ax.set_xlabel(r"Theta / $\theta$")



ax.set_ylabel("Torque / mN·m")

ax.legend(loc = 4,
              ncol = 1,
              columnspacing=0.5,
              handletextpad=0.2,
              fontsize = 8.5,
              handlelength = 1,
              facecolor = 'w',
              framealpha = 1,
              fancybox = False)

plt.show()