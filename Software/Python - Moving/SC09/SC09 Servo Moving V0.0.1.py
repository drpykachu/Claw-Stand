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
stl_path_A = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_A_Python.STL"
stl_path_B = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_B_Python_2.STL"
stl_path_C = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_C_Python.STL"
stl_path_T = r"..\..\..\Hardware\3D Models\Solidworks Servo\SC09\V0.2 - Bushing\Python_STL\SC09_Holder_Motor_T_Python.STL"



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
    set_speed(600, MOTOR_ID) # 100–600 recommended

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
points      = 100  # Number of points for path

minstep     = 300/1023   # stepper motor step angle
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

# Plot Settings
path_colors = ['tab:green','red','tab:orange','cyan','magenta']
joint_colors = ['tab:blue', '#BFBFBF', '#808080', '#404040', '#000000']
rose_color = 'grey'
tex_off = 10 # tet distance away from point
LW = 3 # Line width
PS = 1 # Point Size

vector_x = (1,0,0)
vector_y = (0,1,0)
vector_z = (0,0,1)
# ================= PyVista / Qt ===================

app = QApplication(sys.argv)
main_window = QMainWindow()
main_window.setGeometry(0, 0, pyautogui.size()[0]//2, pyautogui.size()[1])  # Left half of screen
main_window.setWindowTitle("Claw Animation")

plotter = QtInteractor(main_window)
main_window.setCentralWidget(plotter)
# viewing angle (all commented = trimetric)
# plotter.view_zx()  
# plotter.view_yx()
# plotter.view_yz()  

# === Stagnant Plots ===

# Plotting target Circle
circ_tar_points = np.column_stack((circle_tar[0], circle_tar[1], circle_tar[2]*np.ones(len(circle_tar[1]))))
plotter.add_lines(circ_tar_points, width=LW, color='black')

# Plotting Base Circle
base_circle = circle(Offset_R, 0, np.linspace(0,359,100))
base_circle_points = np.column_stack((base_circle[0], base_circle[1], base_circle[2]*np.ones(len(base_circle[1]))))
plotter.add_lines(base_circle_points, width=LW, color='blue')

# Directional rose
limits = H_tar
lims = np.array(range(-limits, limits))
rose_points = np.column_stack((lims, lims*0, lims*0)); plotter.add_lines(rose_points, width=LW, color=rose_color) # X
rose_points = np.column_stack((lims*0, lims, lims*0)); plotter.add_lines(rose_points, width=LW, color=rose_color) # Y
lims = np.array(range(0, limits))
rose_points = np.column_stack((lims*0, lims*0, lims)); plotter.add_lines(rose_points, width=LW, color=rose_color) # Z
plotter.show_bounds(location='back',
                    axes_ranges=[-limits, limits, -limits, limits, -limits/10, limits],
                    font_size=7,
                    xtitle='X',
                    ytitle='Y',
                    ztitle='Z')

# === Moving points ===

# Points
points_poly_dict = {}
points_actor_dict = {}
for p in range(num_fingers):
    for j in range(5):
        points_poly_dict[f'F{p}J{j}'] = pv.PolyData([0.0, 0.0, 0.0])
        if j != 4:
            points_actor_dict[f'F{p}J{j}'] = plotter.add_mesh(points_poly_dict[f'F{p}J{j}'], color=joint_colors[j], point_size=PS, render_points_as_spheres=True)
        else:
            points_actor_dict[f'F{p}J{j}'] = plotter.add_mesh(points_poly_dict[f'F{p}J{j}'], color=path_colors[p], point_size=PS, render_points_as_spheres=True)
            
# Lines - Fingers and Paths lines_poly_dict[f'P{k}'].points = np.column_stack((master_path_plot[0], master_path_plot[1], master_path_plot[2]))
            
lines_poly_dict = {}
lines_actor_dict = {}
for p in range(num_fingers):
    lines_poly_dict[f'F{p}'] = pv.PolyData(np.array([[0.0,0.0,0.0]]*5), lines=np.hstack([[5, *range(5)]]))
    lines_actor_dict[f'F{p}'] = plotter.add_mesh(lines_poly_dict[f'F{p}'], color='black', line_width=LW)

    lines_poly_dict[f'P{p}'] = pv.PolyData(np.array([[0.0,0.0,0.0]]*points), lines=np.hstack([[points, *range(points)]]))
    lines_actor_dict[f'P{p}'] = plotter.add_mesh(lines_poly_dict[f'P{p}'], color=path_colors[p], line_width=LW)


### STL (testing)
motor_poly_dict = {}
motor_actor_dict = {}
opacity_stl = 1
opacity_color = "lightgray"

# Motor A
for p in range(num_fingers):
    mesh = trimesh.load_mesh(stl_path_A)
    pointers, faces = mesh.vertices, mesh.faces
    motor_poly_dict[f'F{p}M{0}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(motor_poly_dict[f'F{p}M{0}'], color=opacity_color, opacity=opacity_stl)
    translate_object(motor_poly_dict[f'F{p}M{0}'], (-0.75,0,0))        # Centers object
    translate_object(motor_poly_dict[f'F{p}M{0}'], (-74.3071,0,0))        # Centers object
    rotate_around_line(motor_poly_dict[f'F{p}M{0}'], (0,0,0), (0,0,1), -90) # Sets the position correctly
    rotate_around_line(motor_poly_dict[f'F{p}M{0}'], (0,0,0), vector_z, -Offset_theta_master[p]) # Sets the position correctly

# Motor B

for p in range(num_fingers):
    mesh = trimesh.load_mesh(stl_path_B)
    pointers, faces = mesh.vertices, mesh.faces
    motor_poly_dict[f'F{p}M{1}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(motor_poly_dict[f'F{p}M{1}'], color=opacity_color, opacity=opacity_stl)
    translate_object(motor_poly_dict[f'F{p}M{1}'], (-45.5/2,-17.375,0))        # Centers object
#     rotate_around_line(motor_poly_dict[f'F{p}M{1}'], (0,0,0), (0,0,1), -180) # Sets the position correctly
    translate_object(motor_poly_dict[f'F{p}M{1}'], (Offset_R,0,A-6.5))        # Lines up to motor center
    rotate_around_line(motor_poly_dict[f'F{p}M{1}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly

# Motor C

for p in range(num_fingers):
    mesh = trimesh.load_mesh(stl_path_C)
    pointers, faces = mesh.vertices, mesh.faces
    motor_poly_dict[f'F{p}M{2}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(motor_poly_dict[f'F{p}M{2}'], color=opacity_color, opacity=opacity_stl)
    translate_object(motor_poly_dict[f'F{p}M{2}'], (-25.0963/2,-45.5/2,0))        # Centers object
    rotate_around_line(motor_poly_dict[f'F{p}M{2}'], (0,0,0), (0,0,1), -180) # Sets the position correctly
    translate_object(motor_poly_dict[f'F{p}M{2}'], (Offset_R,0,A+B-6.5))        # Centers object
    rotate_around_line(motor_poly_dict[f'F{p}M{2}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly

# Motor T

for p in range(num_fingers):
    mesh = trimesh.load_mesh(stl_path_T)
    pointers, faces = mesh.vertices, mesh.faces
    motor_poly_dict[f'F{p}M{3}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(motor_poly_dict[f'F{p}M{3}'], color=opacity_color, opacity=opacity_stl)
    translate_object(motor_poly_dict[f'F{p}M{3}'], (-25.0963/2,-45.5/2,0))        # Centers object
#     rotate_around_line(motor_poly_dict[f'F{p}M{3}'], (0,0,0), (0,0,1), -180) # Sets the position correctly
    translate_object(motor_poly_dict[f'F{p}M{3}'], (Offset_R,0,A+B+C-6.5))        # Centers object
    rotate_around_line(motor_poly_dict[f'F{p}M{3}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly
    
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


# Initial check - finds max angle and steps

for i in range(points):
    Xtar, Ytar, Ztar = master_path[0, :, int(i)]
    try:
        theta_a,theta_b,theta_c = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T,Offset_R)[0] 
        minmax_a[i] = theta_a
        minmax_b[i] = theta_b
        minmax_c[i] = theta_c
    except:
        print('WARNING - not all points in path have a solution.')

theta_reals = np.ones(3)*361.0 # random seed greater than any angle
theta_a = 361.0
theta_b = 361.0
theta_c = 361.0

camera_distance(plotter, distance = 1000)
step_counter = np.zeros(3)*0

def animate():
    global val, theta_a, theta_b, theta_c ,step_counter

    if val == points:
        val = 0
#         print(step_counter)

        
    for k in range(num_fingers):
        try:
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

            ########## MOVES MOTORS ##########
            
            if k == 0:
#                 print(theta_b)
#                 print(360-(np.rad2deg(theta_c) - np.rad2deg(theta_b)+90)-90)
                      
                move(angle360_to_pos(360-(np.rad2deg(theta_a)+90-17)),2)
                move(angle360_to_pos(np.rad2deg(theta_b)+90),3)
                move(angle360_to_pos(360-(np.rad2deg(theta_c) - np.rad2deg(theta_b)+90)-90),4)
#                 input()
#             print(np.rad2deg(theta_reals[2]))
            ##################################

            coords_unr = point_coords(theta_a,theta_b,theta_c,Offset_R, A, B, C, T)
            coords = rotate_vector(coords_unr, Offset_theta)

            # finds delta_thetas
            delta_theta[:,k] = np.array([theta_a, theta_b, theta_c]) - delta_theta[:,k]
              
            
            ### Setting values to actors
            for j in range(5):        
                points_poly_dict[f'F{k}J{j}'].points = np.array([coords[0, j], coords[1, j], coords[2, j]])
            
            lines_poly_dict[f'F{k}'].points = np.column_stack((coords[0, :], coords[1, :], coords[2, :]))
                
            master_path_plot = rotate_vector(master_path[k], Offset_theta)
            lines_poly_dict[f'P{k}'].points = np.column_stack((master_path_plot[0], master_path_plot[1], master_path_plot[2]))
             
            ############ Rotates Motor B Segment ############
            
            # Casts it back to center for easier rotational math
            rotate_around_line(motor_poly_dict[f'F{k}M{1}'], (0,0,0), vector_z, -Offset_theta_master[k]) # Sets the position correctly
            translate_object(motor_poly_dict[f'F{k}M{1}'], (-Offset_R,0,0))        # Lines up to motor center

            if primerB[k] != 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{1}'], point1, vector_x, np.rad2deg(delta_theta[0,k])) # Flips to correct side
            if primerB[k] == 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{1}'], point1, vector_x, np.rad2deg(theta_a)-90) # Flips to correct side
                primerB[k] = 1
                
            # Sends it back out to desired location for easier rotational math
            translate_object(motor_poly_dict[f'F{k}M{1}'], (Offset_R,0,0))        # Lines up to motor center
            rotate_around_line(motor_poly_dict[f'F{k}M{1}'], (0,0,0), vector_z, Offset_theta_master[k]) # Sets the position correctly
            
            ############ Rotates Motor C Segment ############

            rotate_around_line(motor_poly_dict[f'F{k}M{2}'], (0,0,0), vector_z, -Offset_theta_master[k]) 
            translate_object(motor_poly_dict[f'F{k}M{2}'], (-Offset_R,0,0))
            
            if primerC[k] != 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point1, vector_x, -(np.rad2deg(theta_a)-90))
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point2, vector_y, -np.rad2deg(delta_theta[1,k]))
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point1, vector_x, np.rad2deg(delta_theta[0,k]))
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point1, vector_x, np.rad2deg(theta_a)-90)
                
            if primerC[k] == 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point2, vector_y, 90-np.rad2deg(theta_b))
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point1, vector_x, np.rad2deg(theta_a)-90)
                primerC[k] = 1

            translate_object(motor_poly_dict[f'F{k}M{2}'], (Offset_R,0,0))        # Lines up to motor center
            rotate_around_line(motor_poly_dict[f'F{k}M{2}'], (0,0,0), vector_z, Offset_theta_master[k]) # Sets the position correctly

            ############ Rotates Motor T Segment ############
            rotate_around_line(motor_poly_dict[f'F{k}M{3}'], (0,0,0), vector_z, -Offset_theta_master[k]) 
            translate_object(motor_poly_dict[f'F{k}M{3}'], (-Offset_R,0,0))

            if primerT[k] != 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point1, vector_x, -(np.rad2deg(theta_a)-90))
                translate_object(motor_poly_dict[f'F{k}M{3}'], (-dtx[k],0,dtz[k]))
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point3, vector_y, -np.rad2deg(delta_theta[2,k]))
                
                tx, tz  = rotate_point((0,A+B), (0,A+B+C), 90-np.rad2deg(theta_b))
                dtx[k] = 0 - tx 
                dtz[k] = (A+B+C) - tz
                
                translate_object(motor_poly_dict[f'F{k}M{3}'], (dtx[k],0,-dtz[k]))
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point1, vector_x, np.rad2deg(delta_theta[0,k]))
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point1, vector_x, np.rad2deg(theta_a)-90)
                
            if primerT[k] == 0:

                                
                # Initial tilt
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point2, vector_y, 90-np.rad2deg(theta_b))
                tx, tz  = rotate_point((0,A+B), (0,A+B+C), 90-np.rad2deg(theta_b))
                
                dtx[k] = 0 - tx 
                dtz[k] = (A+B+C) - tz                 
                translate_object(motor_poly_dict[f'F{k}M{3}'], (-dtx[k],0,dtz[k]))
                
                thet_d = np.rad2deg(theta_b) - np.rad2deg(theta_c)
                theta_c2 = np.rad2deg(theta_c) - thet_d
                theta_c3 = 90 - theta_c2
                offset = np.rad2deg(theta_c) - np.rad2deg(theta_b)+theta_c3
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point3, vector_y, (theta_c3-offset))
                translate_object(motor_poly_dict[f'F{k}M{3}'], (dtx[k],0,-dtz[k]))

                
                rotate_around_line(motor_poly_dict[f'F{k}M{3}'], point1, vector_x, np.rad2deg(theta_a)-90)
                primerT[k] = 1
                
            translate_object(motor_poly_dict[f'F{k}M{3}'], (Offset_R,0,0))        # Lines up to motor center
            rotate_around_line(motor_poly_dict[f'F{k}M{3}'], (0,0,0), vector_z, Offset_theta_master[k]) # Sets the position correctly

            ################################################
            
            delta_theta[:,k] = np.array([theta_a, theta_b, theta_c])

        except:
            print('Error: no solution geometries found.', end='\r')
            
    plotter.render()
    
    val += 1

timer = QTimer()
timer.timeout.connect(animate)
timer.start(speed) # set speed in ms

# === Show window ===
main_window.show()
sys.exit(app.exec_())
portHandler.closePort()

                                           