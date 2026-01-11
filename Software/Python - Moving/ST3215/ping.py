from st3215 import ST3215
import time


servo = ST3215('COM15')

servo_list = [1,2,3,4,5,6]
servo_here = []
print('Available Servos:')
for servo_ping in servo_list:
    if servo.PingServo(servo_ping): 
        servo_here.append(servo_ping)
print(servo_here)