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

# plot colors
path_colors = ['tab:green','red','tab:orange','cyan','magenta']
joint_colors = ['#FFFFFF', '#BFBFBF', '#808080', '#404040', '#000000']
rose_color = 'grey'

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

# === Moving line ===
line_test2 = np.array([[1.0, 0.0, 1.0], [5.0, 5.0, 10.0]])
line_poly = pv.PolyData(line_test2)
line_actor = plotter.add_mesh(line_poly, color='black', point_size=10, render_points_as_spheres=True)

offset = 0.0
direction = 1
val = 0
delta_theta = np.zeros((3, num_fingers))

# === Stagnant Plots ===

# Plotting target Circle
circ_tar_points = np.column_stack((circle_tar[0], circle_tar[1], circle_tar[2]*np.ones(len(circle_tar[1]))))
plotter.add_lines(circ_tar_points, width=2, color='red')

# Plotting Base Circle
base_circle = circle(Offset_R, 0, np.linspace(0,359,100))
base_circle_points = np.column_stack((base_circle[0], base_circle[1], base_circle[2]*np.ones(len(base_circle[1]))))
plotter.add_lines(base_circle_points, width=2, color='blue')

# Directional rose
limits = 50
lims = np.array(range(-limits, limits))
rose_points = np.column_stack((lims, lims*0, lims*0)); plotter.add_lines(rose_points, width=2, color=rose_color) # X
rose_points = np.column_stack((lims*0, lims, lims*0)); plotter.add_lines(rose_points, width=2, color=rose_color) # Y
lims = np.array(range(-int(limits/10), limits+1))
rose_points = np.column_stack((lims*0, lims*0, lims)); plotter.add_lines(rose_points, width=2, color=rose_color) # Z
plotter.show_bounds(location='back',
                    axes_ranges=[-limits, limits, -limits, limits, -limits/10, limits],
                    font_size=7,
                    xtitle='X',
                    ytitle='Y',
                    ztitle='Z')



# === Animation ===
def animate():
    global offset, direction, line_poly, val
    
    offset += 0.1 * direction
    if offset > 5 or offset < 0:
        direction *= -1

    new_points = line_test2 + np.array([offset, 0.0, 0.0])
    line_poly.points = new_points
    plotter.render()
    val += 1

timer = QTimer()
timer.timeout.connect(animate)
timer.start(50)

# === Show window ===
main_window.show()
sys.exit(app.exec_())
