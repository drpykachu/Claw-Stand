from st3215 import ST3215
import time


servo = ST3215('COM15')

servo_ID = 52   # range(0,100)

servo.MoveTo(servo_ID, 2125, speed = 4000)