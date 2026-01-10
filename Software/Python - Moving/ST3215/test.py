from st3215 import ST3215


servo = ST3215('COM15')

# ids = servo.ListServos()
servo.PingServo(4)

count = 2048
servo.MoveTo(4, count)