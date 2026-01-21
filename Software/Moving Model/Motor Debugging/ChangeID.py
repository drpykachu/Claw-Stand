from st3215 import ST3215
import time


servo = ST3215('COM15')

sts_id = 3
new_id = 12

if servo.PingServo(sts_id) == False:
    print('No unID\'d servo available')

servo.ChangeId(sts_id, new_id)
time.sleep(3)

if servo.PingServo(new_id) == True:
    print(f'Servo ID changed from {sts_id} to {new_id}!')

