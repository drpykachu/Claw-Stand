import sys
import pyvista as pv
from pyvistaqt import QtInteractor
import trimesh
import numpy as np
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

# === Create QApplication first ===
app = QApplication(sys.argv)

# === QtInteractor plotter ===
plotter = QtInteractor()

# Moving line: create PolyData first
line_test2 = np.array([[1.0, 0.0, 1.0], [5.0, 5.0, 10.0]])
line_poly = pv.PolyData(line_test2)
line_actor = plotter.add_mesh(line_poly, color='blue', line_width=5)

offset = 0.0
direction = 1
val = 0

def animate():
    global offset, direction, line_poly, line_actor, val
    offset += 0.1 * direction
    if offset > 5 or offset < 0:
        direction *= -1

    # Update PolyData points
    new_points = line_test2 + np.array([offset, 0.0, 0.0])
    line_poly.points = new_points
    plotter.render()
    val = val + 1
    
    
# === QTimer setup ===
timer = QTimer()
timer.timeout.connect(animate)
timer.start(50)

# === Show window and start Qt loop ===


plotter.show()
sys.exit(app.exec_())

