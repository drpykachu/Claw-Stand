import sys
import vtk
import time
import trimesh
import itertools
import pyautogui  
import numpy as np
import pyvista as pv
from st3215 import ST3215
from PyQt5.QtCore import QTimer
from pyvistaqt import QtInteractor
from PyQt5.QtWidgets import QApplication, QMainWindow
from Claw_Functions_ST3215 import * # Home brew package

# ======================================================= File Imports =========================================================
version = 'V0.6'
stl_path_A  = r"..\..\..\Hardware\3D Models\ST3215\\" + version + r"\Python_STL\ST3215_Motor_A.STL"
stl_path_Ax = r"..\..\..\Hardware\3D Models\ST3215\\" + version + r"\Python_STL\ST3215_Motor_A_Arm.STL"
stl_path_B  = r"..\..\..\Hardware\3D Models\ST3215\\" + version + r"\Python_STL\ST3215_Motor_B.STL"
stl_path_C  = r"..\..\..\Hardware\3D Models\ST3215\\" + version + r"\Python_STL\ST3215_Motor_C.STL"

stl_path_Plate1 = r"..\..\..\Hardware\3D Models\Items\Plate_1.STL"
stl_path_Plate2 = r"..\..\..\Hardware\3D Models\Items\Plate_2.STL"
stl_path_Baseball = r"..\..\..\Hardware\3D Models\Items\Baseball.STL"
items_on = False
# ======================================================= Parameters =========================================================

# Digit Lengths
A = 45 # bottom digit + motor A height
B = 19.8 # lower-middle digit + motor B height

C = 75 # upper-middle middle 2 digit + motor B height
T = 65 # top digit height

A_x = 39.8

num_fingers = 5

Offset_R    = 65.075 # From origin (0,0,0)

R_tar       = 80  # Radius of target circle path 
H_tar       = 190  # Height of target circle path

minstep    = 360/4096*1

delta_Z     = 20   # Dropping from path for reset 
points      = 200  # Number of points for path

speed       = 100   # Animation speed

# ================= Pathing ===================
Offset_theta_master = np.linspace(360,0,num_fingers+1)[0:num_fingers] # Flip the 0 and 360 to change direction
circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, points))        # Builds target circle
fingertip_path = pathing(points, delta_Z,num_fingers,R_tar, H_tar)    # Builds a fingertip path
master_path = patterns('Roll',num_fingers, points,fingertip_path)

# ======================================================= Motor Connections =========================================================

try:
    # Establishes motor communication
    servo = ST3215('COM15')
    servo_dict = {}
    for i in range(num_fingers):
        servo_dict[f'F{i+1}_A'] = 10*i+0+10
        servo_dict[f'F{i+1}_B'] = 10*i+1+10
        servo_dict[f'F{i+1}_C'] = 10*i+2+10
    
    servo_here = []
    print('Available Servos:')
    for servo_ping in servo_dict:
        if servo.PingServo(servo_dict[servo_ping]): 
            servo_here.append(servo_ping)
    print(servo_here)
    
    #### Centers Motors ####
    center_A_C = 2048
    center_B   = 2048
    for motors in servo_here:
        if motors[-1] == 'B':
            servo.MoveTo(servo_dict[motors], center_B)
        else:
            servo.MoveTo(servo_dict[motors], center_A_C)
            
    #### Sets Motor Speed ####
    motor_speed = 4096
    for motors in servo_here:
        servo.SetSpeed(servo_dict[motors], motor_speed)
    motors_found = True

except:
    print('No motors available')
    motors_found = False


# ======================================================= PyVista / Qt =========================================================
vector_x = (1,0,0)
vector_y = (0,1,0)
vector_z = (0,0,1)
app = QApplication(sys.argv)
main_window = QMainWindow()
main_window.setGeometry(0, 0, pyautogui.size()[0]//2, pyautogui.size()[1])  # Left half of screen
main_window.setWindowTitle("Claw Animation")

plotter = QtInteractor(main_window)
main_window.setCentralWidget(plotter)

# === Stagnant Plots ===

# Plotting target Circle
circ_tar_points = np.column_stack((circle_tar[0], circle_tar[1], circle_tar[2]*np.ones(len(circle_tar[1]))))
plotter.add_lines(circ_tar_points, width=3, color='black')

# Plotting Base Circle
base_circle = circle(Offset_R, 0, np.linspace(0,359,100))
base_circle_points = np.column_stack((base_circle[0], base_circle[1], base_circle[2]*np.ones(len(base_circle[1]))))
plotter.add_lines(base_circle_points, width=3, color='blue')

# Directional rose
limits = H_tar
lims = np.array(range(-limits, limits))
rose_points = np.column_stack((lims, lims*0, lims*0)); plotter.add_lines(rose_points, width=3, color='grey') # X
rose_points = np.column_stack((lims*0, lims, lims*0)); plotter.add_lines(rose_points, width=3, color='grey') # Y
lims = np.array(range(0, limits))
rose_points = np.column_stack((lims*0, lims*0, lims)); plotter.add_lines(rose_points, width=3, color='grey') # Z
plotter.show_bounds(location='back',
                    axes_ranges=[-limits, limits, -limits, limits, -limits/10, limits],
                    font_size=7,
                    xtitle='X',
                    ytitle='Y',
                    ztitle='Z')

# === Moving points ===
pyvista_poly_dict = {}
pyvista_actor_dict = {}

pyvista_poly_dict_BAS = {}
pyvista_actor_dict_BAS = {}

opacity_stl = 1
opacity_color = "lightgray"
opacity_item = items_on
path_colors = ['tab:green','red','tab:orange','cyan','magenta']
joint_colors = ['tab:blue', '#BFBFBF','magenta', '#808080', '#404040', '#000000']


for p in range(num_fingers):
    for j in range(6):
        pyvista_poly_dict_BAS[f'F{p}J{j}'] = pv.PolyData([0.0, 0.0, 0.0])
        if j != 5:
            pyvista_actor_dict_BAS[f'F{p}J{j}'] = plotter.add_mesh(pyvista_poly_dict_BAS[f'F{p}J{j}'], color=joint_colors[j], point_size=10, render_points_as_spheres=True)
        else:
            pyvista_actor_dict_BAS[f'F{p}J{j}'] = plotter.add_mesh(pyvista_poly_dict_BAS[f'F{p}J{j}'], color=path_colors[p], point_size=10, render_points_as_spheres=True)

    # Lines - Fingers and Paths 
    pyvista_poly_dict_BAS[f'F{p}'] = pv.PolyData(np.array([[0.0,0.0,0.0]]*6), lines=np.hstack([[6, *range(6)]]))
    pyvista_actor_dict_BAS[f'F{p}'] = plotter.add_mesh(pyvista_poly_dict_BAS[f'F{p}'], color='black', line_width=3)
    pyvista_poly_dict_BAS[f'P{p}'] = pv.PolyData(np.array([[0.0,0.0,0.0]]*points), lines=np.hstack([[points, *range(points)]]))
    pyvista_actor_dict_BAS[f'P{p}'] = plotter.add_mesh(pyvista_poly_dict_BAS[f'P{p}'], color=path_colors[p], line_width=3)

    # Motor A
    mesh = trimesh.load_mesh(stl_path_A) 
    pointers, faces, extents = mesh.vertices, mesh.faces, mesh.extents
    pyvista_poly_dict[f'F{p}M{0}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'F{p}M{0}'], color=opacity_color, opacity=opacity_stl)
    translate_object(pyvista_poly_dict[f'F{p}M{0}'], (0,-extents[1]/2-0.5,0))        # Centers object
    rotate_around_line(pyvista_poly_dict[f'F{p}M{0}'], (0,0,0), vector_z, -Offset_theta_master[p]) # Sets the position correctly

    # Motor A Arm
    mesh = trimesh.load_mesh(stl_path_Ax)
    pointers, faces, extents = mesh.vertices, mesh.faces, mesh.extents
    pyvista_poly_dict[f'F{p}M{1}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'F{p}M{1}'], color=opacity_color, opacity=opacity_stl)
    translate_object(pyvista_poly_dict[f'F{p}M{1}'], (-extents[0]/2,-23.25,0))        # Centers object
    translate_object(pyvista_poly_dict[f'F{p}M{1}'], (Offset_R+4,0,A-33.7))        # Lines up to motor center
    rotate_around_line(pyvista_poly_dict[f'F{p}M{1}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly

    # Motor B
    mesh = trimesh.load_mesh(stl_path_B)
    pointers, faces, extents = mesh.vertices, mesh.faces, mesh.extents
    pyvista_poly_dict[f'F{p}M{2}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'F{p}M{2}'], color=opacity_color, opacity=opacity_stl)
    translate_object(pyvista_poly_dict[f'F{p}M{2}'], (-extents[0]/2,-extents[1]/2,0))        # Centers object
    translate_object(pyvista_poly_dict[f'F{p}M{2}'], (Offset_R+A_x+5,0,A+B-12))        # Centers object
    rotate_around_line(pyvista_poly_dict[f'F{p}M{2}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly

    # Motor C
    mesh = trimesh.load_mesh(stl_path_C)
    pointers, faces, extents = mesh.vertices, mesh.faces, mesh.extents
    pyvista_poly_dict[f'F{p}M{3}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'F{p}M{3}'], color=opacity_color, opacity=opacity_stl)
    translate_object(pyvista_poly_dict[f'F{p}M{3}'], (-extents[0]/2,-extents[1]/2,0))      # Centers object
    translate_object(pyvista_poly_dict[f'F{p}M{3}'], (Offset_R+A_x+5,0,A+B+C-12))        # Centers object
    rotate_around_line(pyvista_poly_dict[f'F{p}M{3}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly

####### Items
if items_on:
    # Plate
    plate_D = 100
    mesh = trimesh.load_mesh(stl_path_Plate1)
    pointers, faces = mesh.vertices, mesh.faces
    pyvista_poly_dict[f'Plate1'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'Plate1'], color=opacity_color, opacity=opacity_item)
    translate_object(pyvista_poly_dict[f'Plate1'], (-plate_D,-plate_D,H_tar))        # Centers object
    mesh = trimesh.load_mesh(stl_path_Plate2)
    pointers, faces = mesh.vertices, mesh.faces
    pyvista_poly_dict[f'Plate2'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'Plate2'], color='k', opacity=opacity_item)
    translate_object(pyvista_poly_dict[f'Plate2'], (-plate_D,-plate_D*2**0.5/2,H_tar))        # Centers object

    # Baseball
    mesh = trimesh.load_mesh(stl_path_Baseball)
    pointers, faces = mesh.vertices, mesh.faces
    pyvista_poly_dict[f'Baseball'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(pyvista_poly_dict[f'Baseball'], color='tab:orange', opacity=opacity_item)
    translate_object(pyvista_poly_dict[f'Baseball'], (0,0, 37.5 + 5 +H_tar))        # Centers object
    
# ============================================================ Animation ============================================================

delta_theta = np.zeros((3, num_fingers))

primerB = np.zeros((num_fingers))
primerC = np.zeros((num_fingers))
primerT = np.zeros((num_fingers))



point1 = (0,0,A)
point2 = (0, 0, A+B) # where the B joint is
point3 = (0, 0, A+B+C) # where the C joint is

dtx = np.zeros((num_fingers))
dtz = np.zeros((num_fingers))

minmax_a = []
minmax_b = []
minmax_c = []
# Initial check - finds max angle and steps

for i in range(points):
    Xtar, Ytar, Ztar = master_path[0, :, int(i)]
    try:
        theta_a,theta_b,theta_c = solve_thetas(Ztar, Ytar, Xtar, A, A_x, B, C, T,Offset_R)[0]
        minmax_a.append(theta_a)
        minmax_b.append(theta_b)
        minmax_c.append(theta_c)
    except Exception as e:
        print('\033[91m EXITING - not all points in path have a solution.\033[0m')
        sys.exit(1)


# print(round(np.rad2deg(max(minmax_a) - min(minmax_a))/minstep))
# print(round(np.rad2deg(max(minmax_b) - min(minmax_b))/minstep))
# print(round(np.rad2deg(max(minmax_c) - min(minmax_c))/minstep))

camera_distance(plotter, distance = 1000)




val_anim = 0
val_motor = 0
# ============================================================ Collision ============================================================
pyvista_poly_dict_collision = {}
# for p in range(num_fingers):
#     pyvista_poly_dict_collision[f'F{p}M{3}'] = pyvista_poly_dict[f'F{p}M{3}']
    
theta_reals = np.ones(3)*361.0 # random seed greater than any angle
theta_a = 361.0
theta_b = 361.0
theta_c = 361.0

def animate():
    global val_anim, val_motor, theta_a, theta_b, theta_c

    #### Updates Animation
    # resets animation loop
    if val_anim == points:
        val_anim = 0
        
    # for items
    if items_on == True:
        rotate_around_line(pyvista_poly_dict[f'Plate1'], (0,0,0), vector_z,360/(points*(num_fingers+1) )) # Sets the position correctly
        rotate_around_line(pyvista_poly_dict[f'Plate2'], (0,0,0), vector_z,360/(points*(num_fingers+1) )) # Sets the position correctly
        rotate_around_line(pyvista_poly_dict[f'Baseball'], (0,0,0), vector_z,360/(points*(num_fingers+1) )) # Sets the position correctly


    ok, collision = check_no_intersections(pyvista_poly_dict_collision)
    if not ok:
        print(f"\033[91mWARNING: Collision between {collision[0]} and {collision[1]}\033[0m\r", end="\r", flush=True)
    else:
        print(f"                                                                                                                        \r", end="\r", flush=True)
        
    for k in range(num_fingers):

        ### Finding angles and rebuilding fingers
        Xtar, Ytar, Ztar = master_path[k, :, int(val_anim)] 
        Offset_theta = Offset_theta_master[k]
        Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar),Offset_theta)
        Ztar_new = Ztar
        
#         theta_a, theta_b, theta_c = solve_thetas(Ztar, Ytar, Xtar, A, A_x, B, C, T,Offset_R)[0]
        
        theta_reals = solve_thetas(Ztar, Ytar, Xtar, A, A_x, B, C, T,Offset_R)[0]
        
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

        err = theta_reals[1] - theta_b
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_b += step_change

        err = theta_reals[2] - theta_c
        if abs(np.rad2deg(err)) > minstep:
            nsteps = int(np.ceil(abs(np.rad2deg(err)) / minstep))
            step_change = np.sign(err) * nsteps * np.deg2rad(minstep)
            theta_c += step_change
        
        
        
        coords_unr = point_coords(theta_a,theta_b,theta_c,Offset_R, A, A_x, B, C, T)
        coords = rotate_vector(coords_unr, Offset_theta)
        
        # Finds change in theta
        delta_theta[:,k] = np.array([theta_a, theta_b, theta_c]) - delta_theta[:,k]
          
        
       ############ Ball and Stick Actors ############
        for j in range(6):        
            pyvista_poly_dict_BAS[f'F{k}J{j}'].points = np.array([coords[0, j], coords[1, j], coords[2, j]])
        
        pyvista_poly_dict_BAS[f'F{k}'].points = np.column_stack((coords[0, :], coords[1, :], coords[2, :]))
            
        master_path_plot = rotate_vector(master_path[k], Offset_theta)
        pyvista_poly_dict_BAS[f'P{k}'].points = np.column_stack((master_path_plot[0], master_path_plot[1], master_path_plot[2]))
         
        ############ Rotates Motor Ax Segment ############
        # Casts it back to center for easier rotational math
        rotate_around_line(pyvista_poly_dict[f'F{k}M{1}'], (0,0,0), vector_z, -Offset_theta_master[k]) # Sets the position correctly
        translate_object(pyvista_poly_dict[f'F{k}M{1}'], (-Offset_R,0,0))        # Lines up to motor center

        if primerB[k] != 0:
            rotate_around_line(pyvista_poly_dict[f'F{k}M{1}'], point1, vector_x, np.rad2deg(delta_theta[0,k])) # Flips to correct side
        if primerB[k] == 0:
            rotate_around_line(pyvista_poly_dict[f'F{k}M{1}'], point1, vector_x, np.rad2deg(theta_a)-90) # Flips to correct side
            primerB[k] = 1
            
        # Sends it back out to desired location for easier rotational math
        translate_object(pyvista_poly_dict[f'F{k}M{1}'], (Offset_R,0,0))        # Lines up to motor center
        rotate_around_line(pyvista_poly_dict[f'F{k}M{1}'], (0,0,0), vector_z, Offset_theta_master[k]) # Sets the position correctly
        
        ############ Rotates Motor B Segment ############
        # Casts it back to center for easier rotational math
        rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], (0,0,0), vector_z, -Offset_theta_master[k]) 
        translate_object(pyvista_poly_dict[f'F{k}M{2}'], (-(Offset_R+A_x),0,0))
        
        if primerC[k] != 0:
            rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], point1, vector_x, -(np.rad2deg(theta_a)-90))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], point2, vector_y, -np.rad2deg(delta_theta[1,k]))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], point1, vector_x, np.rad2deg(delta_theta[0,k]))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], point1, vector_x, np.rad2deg(theta_a)-90)
            
        if primerC[k] == 0:
            rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], point2, vector_y, 90-np.rad2deg(theta_b))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], point1, vector_x, np.rad2deg(theta_a)-90)
            primerC[k] = 1

        translate_object(pyvista_poly_dict[f'F{k}M{2}'], ((Offset_R+A_x),0,0))        # Lines up to motor center
        rotate_around_line(pyvista_poly_dict[f'F{k}M{2}'], (0,0,0), vector_z, Offset_theta_master[k]) # Sets the position correctly

        ############ Rotates Motor C Segment ############
        # Casts it back to center for easier rotational math
        rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], (0,0,0), vector_z, -Offset_theta_master[k]) 
        translate_object(pyvista_poly_dict[f'F{k}M{3}'], (-(Offset_R+A_x),0,0))

        if primerT[k] != 0:
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point1, vector_x, -(np.rad2deg(theta_a)-90))
            translate_object(pyvista_poly_dict[f'F{k}M{3}'], (-dtx[k],0,dtz[k]))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point3, vector_y, -np.rad2deg(delta_theta[2,k]))
            
            tx, tz  = rotate_point((0,A+B), (0,A+B+C), 90-np.rad2deg(theta_b))
            dtx[k] = 0 - tx 
            dtz[k] = (A+B+C) - tz
            
            translate_object(pyvista_poly_dict[f'F{k}M{3}'], (dtx[k],0,-dtz[k]))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point1, vector_x, np.rad2deg(delta_theta[0,k]))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point1, vector_x, np.rad2deg(theta_a)-90)
            
        if primerT[k] == 0:
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point2, vector_y, 90-np.rad2deg(theta_b))
            tx, tz  = rotate_point((0,A+B), (0,A+B+C), 90-np.rad2deg(theta_b))
            
            dtx[k] = 0 - tx 
            dtz[k] = (A+B+C) - tz                 
            translate_object(pyvista_poly_dict[f'F{k}M{3}'], (-dtx[k],0,dtz[k]))
            
            thet_d = np.rad2deg(theta_b) - np.rad2deg(theta_c)
            theta_c2 = np.rad2deg(theta_c) - thet_d
            theta_c3 = 90 - theta_c2
            
            offset = np.rad2deg(theta_c) - np.rad2deg(theta_b)+theta_c3
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point3, vector_y, (theta_c3-offset))
            translate_object(pyvista_poly_dict[f'F{k}M{3}'], (dtx[k],0,-dtz[k]))
            rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], point1, vector_x, np.rad2deg(theta_a)-90)
            primerT[k] = 1
            
        translate_object(pyvista_poly_dict[f'F{k}M{3}'], ((Offset_R+A_x),0,0))        # Lines up to motor center
        rotate_around_line(pyvista_poly_dict[f'F{k}M{3}'], (0,0,0), vector_z, Offset_theta_master[k]) # Sets the position correctly

        delta_theta[:,k] = np.array([theta_a, theta_b, theta_c])


    plotter.render()
    val_anim += 1
    
    ################################################
            
    #### Updates Motor movement

    if val_motor == points:
        val_motor = 0
        
    for k in range(num_fingers):
        ### Finding angles and rebuilding fingers
        Xtar, Ytar, Ztar = master_path[k, :, int(val_motor)] 
        Offset_theta = Offset_theta_master[k]
        Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar),Offset_theta)
        Ztar_new = Ztar
        
        theta_a, theta_b, theta_c = solve_thetas(Ztar, Ytar, Xtar, A, A_x, B, C, T,Offset_R)[0]             
        if motors_found and ok:
            servo.MoveTo(servo_dict[f'F{k+1}_C'], ang2bit(180 + (np.rad2deg(theta_c) - np.rad2deg(theta_b))))
            servo.MoveTo(servo_dict[f'F{k+1}_B'], ang2bit((np.rad2deg(theta_b)+90)))
            servo.MoveTo(servo_dict[f'F{k+1}_A'], ang2bit(360-(np.rad2deg(theta_a)+90)))

    val_motor += 1
    

timer = QTimer()
timer.timeout.connect(animate)
timer.start(speed) # set speed in ms

# === Show window ===
main_window.show()
sys.exit(app.exec_())
                                           

