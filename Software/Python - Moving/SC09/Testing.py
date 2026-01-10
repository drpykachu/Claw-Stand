from scservo_sdk import *
import time

DEVICENAME = 'COM15'
BAUDRATE   = 1000000
SCS_ID     = 2
protocol_end = 1

# ---- SCS register addresses ----
GOAL_POSITION_L = 42   # 0x2A

# Position mapping (0–1023 = 0–300°)
CENTER = 512
DELTA_90 = int(1023 * 30 / 300)

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

def angle360_to_pos(angle_deg):
    """
    Map 0–360° command to SCServo position (0–1023),
    clipped to physical 0–300° range.
    """
    angle = max(0, min(360, angle_deg))   # clamp input
    angle_300 = min(angle, 300)           # clamp to servo range

    pos = int(angle_300 * 1023 / 300)
    return pos


# ---- motion ----
move(POS_MIN)
time.sleep(1)
 
move(POS_MAX)
time.sleep(1)

move(CENTER)

portHandler.closePort()
