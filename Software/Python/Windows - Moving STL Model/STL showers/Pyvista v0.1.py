import sys
import pyvista as pv
from pyvistaqt import QtInteractor
import trimesh
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

# === Create QApplication first ===
app = QApplication(sys.argv)

# === Load STL file ===
stl_path = r"C:\Users\Pyka\Desktop\New folder\Motor A Housing.stl"
mesh = trimesh.load_mesh(stl_path)
points, faces = mesh.vertices, mesh.faces
pv_mesh = pv.PolyData(points, np.hstack([np.full((faces.shape[0], 1), 3), faces]))

# === QtInteractor plotter ===
plotter = QtInteractor()
plotter.add_mesh(pv_mesh, color="lightgray", opacity=0.5)

# Static point
point_test = np.array([[1.0, 1.0, 1.0]])
plotter.add_points(point_test, point_size=10, color='red', render_points_as_spheres=True)

# Static line
line_test = np.array([[1.0, 1.0, 1.0], [10.0, 10.0, 10.0]])
plotter.add_lines(line_test, width=15, color='black')

# Moving line: create PolyData first
line_test2 = np.array([[1.0, 0.0, 1.0], [5.0, 5.0, 10.0]])
line_poly = pv.PolyData(line_test2)
line_actor = plotter.add_mesh(line_poly, color='blue', line_width=5)

offset = 0.0
direction = 1

def animate():
    global offset, direction, line_poly, line_actor
    offset += 0.1 * direction
    if offset > 5 or offset < 0:
        direction *= -1

    # Update PolyData points
    new_points = line_test2 + np.array([offset, 0.0, 0.0])
    line_poly.points = new_points
    plotter.render()

# === QTimer setup ===
timer = QTimer()
timer.timeout.connect(animate)
timer.start(50)

# === Show window and start Qt loop ===


plotter.show()
sys.exit(app.exec_())
