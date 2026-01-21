
from st3215 import ST3215
num_fingers = 5
try:
    # Establishes motor communication
    servo = ST3215('COM15')
    servo_dict = {}
    for i in range(num_fingers):
        servo_dict[f'F{i+1}_A'] = 10+i*3
        servo_dict[f'F{i+1}_B'] = 11+i*3
        servo_dict[f'F{i+1}_C'] = 12+i*3

    servo_here = []
    print('Available Servos:')
    for servo_ping in servo_dict:
        if servo.PingServo(servo_dict[servo_ping]): 
            servo_here.append(servo_ping)
    print(servo_here)

    #### Centers Motors ####
    center = 2500
    for motors in servo_here:

        servo.MoveTo(servo_dict[motors], center, speed = 4000)



except:
    print('No motors available')
    motors_found = False