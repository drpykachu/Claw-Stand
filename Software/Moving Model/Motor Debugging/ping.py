from st3215 import ST3215
import time


servo = ST3215('COM15')

servo_list = [10,11,12,13,20,21,22,30,31,32,40,41,42,50,51,52] # range(0,100)

servo_here = []
print('Available Servos:')
for servo_ping in servo_list:
    if servo.PingServo(servo_ping): 
        servo_here.append(servo_ping)
print(servo_here) 