from st3215 import ST3215
import time


servo = ST3215('COM15')

motors = [10,20,30,40,50]
for k in range(len(motors)):
    servo_ID = motors[k]   # range(0,100)
    servo.MoveTo(servo_ID, 2048, speed = 4000)
