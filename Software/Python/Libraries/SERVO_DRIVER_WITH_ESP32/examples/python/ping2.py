
from scservo_sdk import *

DEVICENAME = 'COM15'
BAUDRATE   = 115200
SCS_ID     = 1
protocol_end = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(protocol_end)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

model, comm, err = packetHandler.ping(portHandler, SCS_ID)

print(model, comm, err)

portHandler.closePort()
