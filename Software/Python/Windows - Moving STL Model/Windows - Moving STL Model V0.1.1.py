import sys
import trimesh
import pyautogui  
import numpy as np
import pyvista as pv
import matplotlib.pyplot as plt
from PyQt5.QtCore import QTimer
from pyvistaqt import QtInteractor
import matplotlib.colors as mcolors
from PyQt5.QtWidgets import QApplication, QMainWindow

from Claw_Functions import * # Home brew package


# ================= Parameters ===================

# Digit Lengths
A = 57.38 # bottom digit + motor A height
B = 60 # lower-middle digit + motor B height
C = 60 # upper-middle middle 2 digit + motor B height
T = 60 # top digit height

num_fingers = 5
Offset_R    = 74.05  # From origin (0,0,0)
R_tar       = 120  # Radius of target circle path 
H_tar       = 220  # Height of target circle path

delta_Z     = 20   # Dropping from path for reset 
points      = 100 # Number of points for path 


Offset_theta_master = np.linspace(360,0,num_fingers+1)[0:num_fingers] # Flip the 0 and 360 to change direction
circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, points))        # Builds target circle
fingertip_path = pathing(points, delta_Z,num_fingers,R_tar, H_tar)    # Builds a fingertip path
master_path = np.ones((num_fingers,3,points))                         # Allocates data for shifted finger path



for i in range(num_fingers):
    """ Loop to adjust the finger path for each finger, offset by the points/fingers for even spacing."""
    shifter = int(points / (num_fingers) * i)
    shifted = np.roll(fingertip_path, shift=shifter, axis=1)
    master_path[i] = shifted

# Plot Settings
path_colors = ['tab:green','red','tab:orange','cyan','magenta']
joint_colors = ['tab:blue', '#BFBFBF', '#808080', '#404040', '#000000']
rose_color = 'grey'
tex_off = 10 # tet distance away from point
LW = 3 # Line width
PS = 15 # Point Size


# ================= PyVista / Qt ===================

app = QApplication(sys.argv)
main_window = QMainWindow()
main_window.setGeometry(0, 0, pyautogui.size()[0]//2, pyautogui.size()[1])  # Left half of screen
main_window.setWindowTitle("Claw Animation")

plotter = QtInteractor(main_window)
main_window.setCentralWidget(plotter)
# viewing angle (all commented = trimetric)
plotter.view_zx()  
# plotter.view_yx()
plotter.view_yz()  
camera_distance(plotter, distance = 800 )





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
            
# Lines - Fingers and Paths
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

# Motor A
stl_path_A = r"C:\Users\Pyka\Documents\GitHub\Claw-Stand\Hardware\3D Models\Solidworks V0.1\Sub Assemblies\Motor A + bearing + base.STL"
p = 'ALL'
mesh = trimesh.load_mesh(stl_path_A)
pointers, faces = mesh.vertices, mesh.faces
motor_poly_dict[f'F{p}M{0}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
plotter.add_mesh(motor_poly_dict[f'F{p}M{0}'], color="lightgray", opacity=0.5)
rotate_around_line(motor_poly_dict[f'F{p}M{0}'], (0,0,0), (1,0,0), 90) # Gets the object flat
translate_object(motor_poly_dict[f'F{p}M{0}'], (-118.5,101,0))        # Centers object
rotate_around_line(motor_poly_dict[f'F{p}M{0}'], (0,0,0), (0,0,1), -90) # Sets the position correctly

# Motor B
stl_path_B = r"C:\Users\Pyka\Documents\GitHub\Claw-Stand\Hardware\3D Models\Solidworks V0.1\Sub Assemblies\Motor B + bearing.STL"
for p in range(num_fingers):
    mesh = trimesh.load_mesh(stl_path_B)
    pointers, faces = mesh.vertices, mesh.faces
    motor_poly_dict[f'F{p}M{1}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(motor_poly_dict[f'F{p}M{1}'], color="lightgray", opacity=0.5)
    rotate_around_line(motor_poly_dict[f'F{p}M{1}'], (0,0,0), (1,0,0), 90) # Gets the object Up
    translate_object(motor_poly_dict[f'F{p}M{1}'], (-23,15,0))        # Centers object
    rotate_around_line(motor_poly_dict[f'F{p}M{1}'], (0,0,0), (0,0,1), 180) # Flips to correct side
    translate_object(motor_poly_dict[f'F{p}M{1}'], (Offset_R,0,A-15))        # Lines up to motor center
    rotate_around_line(motor_poly_dict[f'F{p}M{1}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly

# Motor C
stl_path_C = r"C:\Users\Pyka\Documents\GitHub\Claw-Stand\Hardware\3D Models\Solidworks V0.1\Sub Assemblies\Motor C.STL"
for p in range(num_fingers):
    mesh = trimesh.load_mesh(stl_path_C)
    pointers, faces = mesh.vertices, mesh.faces
    motor_poly_dict[f'F{p}M{2}'] = pv.PolyData(pointers, np.hstack([np.full((faces.shape[0], 1), 3), faces]))    
    plotter.add_mesh(motor_poly_dict[f'F{p}M{2}'], color="lightgray", opacity=0.5)
    rotate_around_line(motor_poly_dict[f'F{p}M{2}'], (0,0,0), (1,0,0), 90) # Gets the object Up
    translate_object(motor_poly_dict[f'F{p}M{2}'], (-23,15,0))        # Centers object
    rotate_around_line(motor_poly_dict[f'F{p}M{2}'], (0,0,0), (0,0,1), 90) # Flips to correct side
    translate_object(motor_poly_dict[f'F{p}M{2}'], (Offset_R,0,A-15+B))        # Lines up to motor center
    rotate_around_line(motor_poly_dict[f'F{p}M{2}'], (0,0,0), (0,0,1), Offset_theta_master[p]) # Sets the position correctly


# === Animation ===
delta_theta = np.zeros((3, num_fingers))
delta_pos_B = np.zeros((3, num_fingers))


primerB = np.zeros((num_fingers))
primerC = np.zeros((num_fingers))
primerT = np.zeros((num_fingers))

val = 0
tester = 0
def animate():
    global val, primer, tester

    if val == points:
        val = 0
        
    for k in range(num_fingers):
        try:
            ### Finding angles and rebuilding fingers
            Xtar, Ytar, Ztar = master_path[k, :, int(val)]
            Offset_theta = Offset_theta_master[k]
            Xtar_new, Ytar_new = rotate_point((0,0), (Xtar, Ytar),Offset_theta)
            Ztar_new = Ztar
            Xtar_text = Xtar_new + tex_off * np.cos(np.deg2rad(Offset_theta))
            Ytar_text = Ytar_new + tex_off * np.sin(np.deg2rad(Offset_theta))            

            theta_a,theta_b,theta_c = solve_thetas(Ztar, Ytar, Xtar, A, B, C, T,Offset_R)[0] 

            coords_unr = point_coords(theta_a,theta_b,theta_c,Offset_R, A, B, C, T)
            coords = rotate_vector(coords_unr, Offset_theta)

            # finds delta_thetas
            delta_theta[:,k] = np.array([theta_a, theta_b, theta_c]) - delta_theta[:,k]
            
            # finds delta_positions for motor B, since it rotates and moves
            delta_pos_B[:,k] = np.array([coords_unr[0, 2], coords_unr[1, 2], coords_unr[2, 2]]) - delta_pos_B[:,k]           
            
            
            ### Setting values to actors
            for j in range(5):        
                points_poly_dict[f'F{k}J{j}'].points = np.array([coords[0, j], coords[1, j], coords[2, j]])
            
            lines_poly_dict[f'F{k}'].points = np.column_stack((coords[0, :], coords[1, :], coords[2, :]))
                
            master_path_plot = rotate_vector(master_path[k], Offset_theta)
            lines_poly_dict[f'P{k}'].points = np.column_stack((master_path_plot[0], master_path_plot[1], master_path_plot[2]))
            
            #### Rotates Motor B Segment
            vector = (1,0,0)
            point = (0,0,A)
            
            # Casts it back to center for easier rotational math
            rotate_around_line(motor_poly_dict[f'F{k}M{1}'], (0,0,0), (0,0,1), -Offset_theta_master[k]) # Sets the position correctly
            translate_object(motor_poly_dict[f'F{k}M{1}'], (-Offset_R,0,0))        # Lines up to motor center

            if primerB[k] != 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{1}'], point, vector, np.rad2deg(delta_theta[0,k])) # Flips to correct side
            if primerB[k] == 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{1}'], point, vector, np.rad2deg(theta_a)-90) # Flips to correct side
                primerB[k] = 1
                
            # Sends it back out to desired location for easier rotational math
            translate_object(motor_poly_dict[f'F{k}M{1}'], (Offset_R,0,0))        # Lines up to motor center
            rotate_around_line(motor_poly_dict[f'F{k}M{1}'], (0,0,0), (0,0,1), Offset_theta_master[k]) # Sets the position correctly
            
            #### Rotates Motor C Segment
            vector = (1,0,0)
            point = (0,0,A+B)
            

            # Casts it back to center for easier rotational math
            rotate_around_line(motor_poly_dict[f'F{k}M{2}'], (0,0,0), (0,0,1), -Offset_theta_master[k]) # Sets the position correctly
            translate_object(motor_poly_dict[f'F{k}M{2}'], (-Offset_R,0,0))        # Lines up to motor center
            
            if primerC[k] != 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point, vector, np.rad2deg(delta_theta[0,k])) # Flips to correct side
                translate_object(motor_poly_dict[f'F{k}M{2}'], (0,delta_pos_B[1,k],0))
                # need rotational math for tilt HERE
                
            if primerC[k] == 0:
                rotate_around_line(motor_poly_dict[f'F{k}M{2}'], point, vector, np.rad2deg(theta_a)-90) # Flips to correct side
                translate_object(motor_poly_dict[f'F{k}M{2}'], (0,delta_pos_B[1,k],delta_pos_B[2,k]-A-B))
                # need rotational math for tilt HERE
                
                primerC[k] = 1

            # Sends it back out to desired location for easier rotational math
            translate_object(motor_poly_dict[f'F{k}M{2}'], (Offset_R,0,0))        # Lines up to motor center
            rotate_around_line(motor_poly_dict[f'F{k}M{2}'], (0,0,0), (0,0,1), Offset_theta_master[k]) # Sets the position correctly


    
            delta_theta[:,k] = np.array([theta_a, theta_b, theta_c])
            delta_pos_B[:,k] = np.array([coords_unr[0, 2], coords_unr[1, 2], coords_unr[2, 2]])

        except:
            print('Error: no solution geometries found.', end='\r')
            
    plotter.render()
    
    val += 1
    

timer = QTimer()
timer.timeout.connect(animate)
timer.start(100) # set speed in ms

# === Show window ===
main_window.show()
sys.exit(app.exec_())
