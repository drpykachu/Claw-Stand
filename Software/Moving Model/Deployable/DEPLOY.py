import time
import numpy as np
from st3215 import ST3215
from DEPLOY_FUNCTIONS import * # Home brew package


# ======================================================= Parameters =========================================================

# Digit Lengths
A = 45 # bottom digit + motor A height
A_x = 39.8
B = 19.8 # lower-middle digit + motor B height

C = 75 # upper-middle middle 2 digit + motor B height
T = 65 # top digit height

num_fingers = 5

Offset_R    = 60.075  # From origin (0,0,0)

R_tar       = 80  # Radius of target circle path 
H_tar       = 190  # Height of target circle path


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





val_motor = 0

def animate():
    global val_motor

    if val_motor == points:
        val_motor = 0
            
    #### Updates Motor movement

        
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
             
while True:
    animate()
    time.sleep(1)
    print('testing')
    