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
A = 10 # bottom digit + motor A height
B = 10 # lower-middle digit + motor B height
C = 15 # upper-middle middle 2 digit + motor B height
T = 10 # top digit height

num_fingers = 5
Offset_R    = 20  # From origin (0,0,0)
R_tar       = 35  # Radius of target circle path 
H_tar       = 35  # Height of target circle path

delta_Z     = 5   # Dropping from path for reset 
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
camera_distance(plotter, distance = 350)

# viewing angle (all commented = trimetric)
# plotter.view_zx()  # top-down-ish
# plotter.view_zx()
# plotter.view_zx()



# === Stagnant Plots ===

# Plotting target Circle
circ_tar_points = np.column_stack((circle_tar[0], circle_tar[1], circle_tar[2]*np.ones(len(circle_tar[1]))))
plotter.add_lines(circ_tar_points, width=LW, color='black')

# Plotting Base Circle
base_circle = circle(Offset_R, 0, np.linspace(0,359,100))
base_circle_points = np.column_stack((base_circle[0], base_circle[1], base_circle[2]*np.ones(len(base_circle[1]))))
plotter.add_lines(base_circle_points, width=LW, color='blue')

# Directional rose
limits = 50
lims = np.array(range(-limits, limits))
rose_points = np.column_stack((lims, lims*0, lims*0)); plotter.add_lines(rose_points, width=LW, color=rose_color) # X
rose_points = np.column_stack((lims*0, lims, lims*0)); plotter.add_lines(rose_points, width=LW, color=rose_color) # Y
lims = np.array(range(-int(limits/10), limits+1))
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


# STL
stl_path = r"C:\Users\Pyka\Documents\GitHub\Claw-Stand\Hardware\3D Models\Solidworks V0.1\Sub Assemblies\SubAssem_MotorA.STL"
mesh = trimesh.load_mesh(stl_path)
vertices, faces = mesh.vertices, mesh.faces

# Shift so centroid is at (0,0,0)
center = vertices.mean(axis=0)
vertices -= center

# Convert to PyVista
pv_mesh = pv.PolyData(vertices, np.hstack([np.full((faces.shape[0], 1), 3), faces]))


# === Animation ===
val = 0
def animate():
    global val
    
    for k in range(num_fingers):
        
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
        
        ### Setting values to actors
        for j in range(5):        
            points_poly_dict[f'F{k}J{j}'].points = np.array([coords[0, j], coords[1, j], coords[2, j]])
        
        lines_poly_dict[f'F{k}'].points = np.column_stack((coords[0, :], coords[1, :], coords[2, :]))
            
        master_path_plot = rotate_vector(master_path[k], Offset_theta)
        lines_poly_dict[f'P{k}'].points = np.column_stack((master_path_plot[0], master_path_plot[1], master_path_plot[2]))


    plotter.render()
    
    val += 1
    if val == points:
        val =- 0

timer = QTimer()
timer.timeout.connect(animate)
timer.start(100) # set speed in ms

# === Show window ===
main_window.show()
sys.exit(app.exec_())
