from st3215 import ST3215


servo = ST3215('COM15')

# ids = servo.ListServos()
servo_list = [1,2,3,4,5,6]
servo_here = []
print('Available Servos:')
for servo_ping in servo_list:
    if servo.PingServo(servo_ping): 
        servo_here.append(servo_ping)
print(servo_here)

#### Centers Motors ####
center = 2048
for motors in servo_here:
    servo.MoveTo(motors, center)
    
    
def ang2bit(angle_deg):
    pos = int(4096 * angle_deg / 360)
    return pos

servo.MoveTo(4, ang2bit(270),speed = 1000)