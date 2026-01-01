from scservo_sdk import *
import time

DEVICENAME = 'COM15'
BAUDRATE   = 1000000
SCS_ID     = 1
protocol_end = 1

# ---- SCS register addresses ----
GOAL_POSITION_L = 42   # 0x2A

# Position mapping (0–1023 = 0–300°)
CENTER = 512
DELTA_90 = int(1023 * 90 / 300)

POS_MIN = CENTER - DELTA_90   # -90°
POS_MAX = CENTER + DELTA_90   # +90°

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(protocol_end)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

def move(pos):
    packetHandler.write2ByteTxRx(
        portHandler,
        SCS_ID,
        GOAL_POSITION_L,
        pos
    )

# ---- motion ----
move(POS_MIN)
time.sleep(1)

move(POS_MAX)
time.sleep(1)

move(CENTER)

portHandler.closePort()
