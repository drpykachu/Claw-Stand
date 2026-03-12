
import numpy as np
import matplotlib.pyplot as plt
from Claw_Functions_ST3215 import * # Home brew package


# ======================================================= Parameters =========================================================

# Digit Lengths
A = 45 # bottom digit + motor A height
B = 19.8 # lower-middle digit + motor B height
C = 75 # upper-middle middle 2 digit + motor B height
T = 65 # top digit height
A_x = 39.8
num_fingers = 5
Offset_R    = 60.075  # From origin (0,0,0)
R_tar       = 80  # Radius of target circle path 
H_tar       = 190  # Height of target circle path
delta_Z     = 20   # Dropping from path for reset 
points      = 200  # Number of points for path
time_path = 10 # seconds
# ================= Pathing ===================
Offset_theta_master = np.linspace(360,0,num_fingers+1)[0:num_fingers] # Flip the 0 and 360 to change direction
circle_tar = circle(R_tar, H_tar, np.linspace(0, 359, points))        # Builds target circle
fingertip_path = pathing(points, delta_Z,num_fingers,R_tar, H_tar)    # Builds a fingertip path
master_path = patterns('Roll',num_fingers, points,fingertip_path)



    
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

fig, axs = plt.subplots(1, 3, sharex=True, sharey=True, figsize=(6, 3), constrained_layout=True)
x_range = np.linspace(0,time_path,points)
axs[0].plot(x_range,minmax_a)
axs[1].plot(x_range,minmax_b)
axs[2].plot(x_range,minmax_c)



fig2, axs2 = plt.subplots(1, 3, sharex=True, sharey=True, figsize=(6, 3), constrained_layout=True)

axs2[0].plot(x_range,np.gradient(minmax_a, x_range))
axs2[1].plot(x_range,np.gradient(minmax_b, x_range))
axs2[2].plot(x_range,np.gradient(minmax_c, x_range))

plt.show()



