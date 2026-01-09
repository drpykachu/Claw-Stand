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



def angle360_to_pos(angle_deg):
    """
    Map 0–360° command to SCServo position (0–1023),
    clipped to physical 0–300° range.
    """    
    pos = int(1023 * angle_deg / 360)+1

    return pos

# ================= File Imports ===================
from Claw_Functions import * # Home brew package
stl_path_A = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_A_Python.STL"
stl_path_B = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_B_Python_2.STL"
stl_path_C = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_C_Python.STL"
stl_path_T = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_T_Python.STL"

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
points      = 100  # Number of points for path

minstep     = 300/1023    # stepper motor step angle
speed       = 20   # Animation speed


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

theta_reals = np.ones(3)*361.0 # random seed greater than any angle
theta_a = 361.0
theta_b = 361.0
theta_c = 361.0

step_counter = np.zeros(3)*0

t_a = []
t_b = []
t_c = []

val = 0
while True:

    if val == points:
        val = 0
        print('done')
        break

    for k in range(num_fingers):
        ### Finding angles and rebuilding fingers
        Xtar, Ytar, Ztar = master_path[k, :, int(val)] 
        Offset_theta = Offset_theta_master[k]
        Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar),Offset_theta)
        Ztar_new = Ztar
        
        theta_reals = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T,Offset_R)[0] 
        
        # checks for a change in angle based on the stepper motor step. makes it go from indefinite to a real world
        if theta_a == 361.0:
                theta_a = theta_reals[0]
                theta_b = theta_reals[1]
                theta_c = theta_reals[2]
        
        err = theta_reals[0] - theta_a
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_a += step_change
            step_counter[0] += np.rad2deg(step_change)/minstep

        err = theta_reals[1] - theta_b
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_b += step_change
            step_counter[1] += np.rad2deg(step_change)/minstep

        err = theta_reals[2] - theta_c
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_c += step_change
            step_counter[2] += np.rad2deg(step_change)/minstep

        if k == 0:
            t_a.append(np.rad2deg(theta_a))
            t_b.append(np.rad2deg(theta_b))
            t_c.append(np.rad2deg(theta_b - theta_c))

    
    val += 1
                 

plt.figure()
plt.plot(range(points),t_a,'k')
plt.plot(range(points),t_b,'k')
plt.plot(range(points),t_c,'k')
plt.xlabel('point in operation')
plt.ylabel('degree in operation')



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
points      = 100  # Number of points for path

minstep     = 300/1023*15    # stepper motor step angle
speed       = 20   # Animation speed


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

theta_reals = np.ones(3)*361.0 # random seed greater than any angle
theta_a = 361.0
theta_b = 361.0
theta_c = 361.0

step_counter = np.zeros(3)*0

t_a = []
t_b = []
t_c = []

val = 0
while True:

    if val == points:
        val = 0
        print('done')
        break

    for k in range(num_fingers):
        ### Finding angles and rebuilding fingers
        Xtar, Ytar, Ztar = master_path[k, :, int(val)] 
        Offset_theta = Offset_theta_master[k]
        Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar),Offset_theta)
        Ztar_new = Ztar
        
        theta_reals = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T,Offset_R)[0] 
        
        # checks for a change in angle based on the stepper motor step. makes it go from indefinite to a real world
        if theta_a == 361.0:
                theta_a = theta_reals[0]
                theta_b = theta_reals[1]
                theta_c = theta_reals[2]
        
        err = theta_reals[0] - theta_a
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_a += step_change
            step_counter[0] += np.rad2deg(step_change)/minstep

        err = theta_reals[1] - theta_b
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_b += step_change
            step_counter[1] += np.rad2deg(step_change)/minstep

        err = theta_reals[2] - theta_c
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_c += step_change
            step_counter[2] += np.rad2deg(step_change)/minstep

        if k == 0:
            t_a.append(float(np.round(np.rad2deg(theta_a),3)))
            t_b.append(float(np.round(np.rad2deg(theta_b),3)))
            t_c.append(float(np.round(np.rad2deg(theta_b - theta_c),3)))

    
    val += 1
                 


plt.plot(range(points),t_a,'b',label = 'Motor A')
plt.plot(range(points),t_b,'orange',label = 'Motor B')
plt.plot(range(points),t_c,'green',label = 'Motor C')
# plt.legend()
# plt.show()

time_array_a = []
count_array_a = []

counter = 1

for k in range(points-1):
    if t_a[k] != t_a[k+1]:
        time_array_a.append(t_a[k])
        count_array_a.append(counter)
        counter = 0
        
    counter = counter+1
    
speed_array = []
for k in range(len(time_array_a)-1):
    speed_array.append(float(np.round(abs(time_array_a[k] -  time_array_a[k+1])/count_array_a[k],3)))
    
print(len(time_array_a))

# #####
speeder = 100
#
MOTOR_ID = 2
set_speed(1023, MOTOR_ID) # 100–600 recommended
move(angle360_to_pos(time_array_a[0]),MOTOR_ID)

for k in range(len(speed_array)):
    set_speed(int(speeder/max(speed_array)*speed_array[k]), MOTOR_ID) # 100–600 recommended
    move(angle360_to_pos(time_array_a[k]),MOTOR_ID)

start = time.time()

 
# set_speed(600, 2) # 100–600 recommended
# move(angle360_to_pos(t_a[0]),2)
# time_array_a
# 
                
