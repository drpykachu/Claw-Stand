import sys
import time
import trimesh
import pyautogui  
import numpy as np
import pyvista as pv
from scservo_sdk import *
import matplotlib.pyplot as plt
from PyQt5.QtCore import QTimer
from pyvistaqt import QtInteractor
import matplotlib.colors as mcolors
from PyQt5.QtWidgets import QApplication, QMainWindow

# ================= File Imports ===================
from Claw_Functions import * # Home brew package

# ================= MOTOR CONTROL ===================
# ---- Overall ----

DEVICENAME = 'COM15'
BAUDRATE   = 1000000
protocol_end = 1

MOTOR_ID_LIST = [
    2,3,4 # Finger 1 - 2: bottom, 3: middle, 4: top
    ]


# ---- SCS register addresses ----
GOAL_POSITION_L = 42   # 0x2A
MOVING_SPEED_L = 46 # 0x2E

# Position mapping (0–1023 = 0–300°)
CENTER = 512
DELTA_90 = int(1023 * 30 / 300)

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(protocol_end)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def move(pos,SCS_ID):
    packetHandler.write2ByteTxRx(
        portHandler,
        SCS_ID,
        GOAL_POSITION_L,
        pos
    )

def set_speed(speed, sid):
    packetHandler.write2ByteTxRx(
        portHandler,
        sid,
        MOVING_SPEED_L,
        speed )
    
# INTIAL MOVEMENT
for MOTOR_ID in MOTOR_ID_LIST:
    move(CENTER,MOTOR_ID)

for MOTOR_ID in MOTOR_ID_LIST:
    set_speed(300, MOTOR_ID) # 100–600 recommended

def angle360_to_pos(angle_deg):
    """
    Map 0–360° command to SCServo position (0–1023),
    clipped to physical 0–300° range.
    """    
    pos = int(1023 * angle_deg / 360)+1

    return pos


# ================= Parameters ===================

# Digit Lengths

A = 33.55 # bottom digit + motor A height
B = 43.55 # lower-middle digit + motor B height
C = 43.55 # upper-middle middle 2 digit + motor B height
T = 50 # top digit height

num_fingers = 5
Offset_R    = 75  # From origin (0,0,0)

R_tar       = 100  # Radius of target circle path 
H_tar       = 160  # Height of target circle path

delta_Z     = 20   # Dropping from path for reset 
points      = 200  # Number of points for path

minstep     = 1   # stepper motor step angle
speed       = 0.05   # Animation speed


Offset_theta_master = np.linspace(360,0,num_fingers+1)[0:num_fingers] # Flip the 0 and 360 to change direction
circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, points))        # Builds target circle
fingertip_path = pathing(points, delta_Z,num_fingers,R_tar, H_tar)    # Builds a fingertip path
master_path = np.ones((num_fingers,3,points))                         # Allocates data for shifted finger path

path_load = int(points/num_fingers*(num_fingers-1))
master_path_load = np.concatenate((np.ones((num_fingers,1,path_load)), np.zeros((num_fingers,1,points - path_load))), axis=2)# Allocates data for shifted finger path, load path


############# PATTERNS #############
# # ROLLING PATTERN SHIFT
# for i in range(num_fingers):
#     """ Loop to adjust the finger path for each finger, offset by the points/fingers for even spacing."""
#     shifter = int(points / (num_fingers) * i)
#     shifted = np.roll(fingertip_path, shift=shifter, axis=1)
#     master_path[i] = shifted



# STAR PATTERN SHIFT (e.g. 1→3→5→2→4 for 5 fingers)
# Uses modular stepping to assign shifts in a star order instead of sequential order

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
######################################


# === Animation ===
delta_theta = np.zeros((3, num_fingers))


primerB = np.zeros((num_fingers))
primerC = np.zeros((num_fingers))
primerT = np.zeros((num_fingers))

val = 0

point1 = (0,0,A)
point2 = (0, 0, A+B) # where the B joint is
point3 = (0, 0, A+B+C) # where the C joint is

dtx = np.zeros((num_fingers))
dtz = np.zeros((num_fingers))

minmax_a = np.zeros(points)
minmax_b = np.zeros(points)
minmax_c = np.zeros(points)

theta_reals = np.ones(3)*361.0 # random seed greater than any angle
theta_a = 361.0
theta_b = 361.0
theta_c = 361.0

step_counter = np.zeros(3)*0
# Initialize current angles separately for each joint & finger
current_theta = np.full((3, num_fingers), np.nan)  # nan = uninitialized

while True:
    time.sleep(speed)
    if val == points:
        val = 0

    for k in range(num_fingers):
        try:
            Xtar, Ytar, Ztar = master_path[k, :, val]
            Offset_theta = Offset_theta_master[k]
            Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar), Offset_theta)
            
            # Solve IK for target angles
            theta_target = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T, Offset_R)[0]

            # Initialize if first loop
            if np.isnan(current_theta[0,k]):
                current_theta[:,k] = theta_target

            # Interpolate angles individually
            for joint in range(3):
                err_deg = np.rad2deg(theta_target[joint] - current_theta[joint,k])
                if abs(err_deg) > minstep:
                    step = np.sign(err_deg) * np.deg2rad(minstep)
                    current_theta[joint,k] += step
                else:
                    current_theta[joint,k] = theta_target[joint]

            # Move motors for finger 0 (example)
            if k == 0:
                theta_a, theta_b, theta_c = current_theta[:,0]
                move(angle360_to_pos(360-(np.rad2deg(theta_a)+90-17)), 2)
                move(angle360_to_pos(np.rad2deg(theta_b)+90), 3)
                move(angle360_to_pos(360-(np.rad2deg(theta_c) - np.rad2deg(theta_b)+90)-90), 4)

        except Exception as e:
            print(f'Error: {e}', end='\r')

    val += 1


portHandler.closePort()

                                           

