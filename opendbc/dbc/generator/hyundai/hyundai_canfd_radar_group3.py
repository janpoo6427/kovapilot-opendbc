#!/usr/bin/env python3
# Hyundai/Kia CAN-FD front radar object list ("group 3"), seen on the A-CAN of the Ioniq 9.
# 30 messages, 0x400-0x41D, 24 bytes each, 20 Hz. An address is only a transport slot:
# the same object can move between addresses, OBJECT_ID identifies it across cycles.
# Empty slot: OBJECT_ID 0 and LONG_DIST raw 0x7ff. LAT_DIST is positive to the left.
# Bit positions were checked against Ioniq 9 logs (the CRC16 matches on every frame).
# The counter is not named COUNTER on purpose: while driving it repeats and then skips by 2 on some slots
# (logged 0000079b--11, 00000798--62), which the CAN parser counter check would report as a CAN error.

GROUP3_START = 0x400
GROUP3_COUNT = 30


def generate():
  parts = ["""VERSION ""


NS_ :
    CM_
    BA_DEF_
    BA_
    VAL_

BS_:

BU_: XXX RADAR
"""]
  for addr in range(GROUP3_START, GROUP3_START + GROUP3_COUNT):
    parts.append(f"""
BO_ {addr} RADAR_GROUP3_{addr:x}: 24 RADAR
 SG_ CHECKSUM : 0|16@1+ (1,0) [0|65535] "" XXX
 SG_ FRAME_COUNTER : 16|8@1+ (1,0) [0|255] "" XXX
 SG_ OBJECT_ID : 24|7@1+ (1,0) [0|127] "" XXX
 SG_ OBJECT_LENGTH : 41|7@1+ (0.1,0) [0|12.7] "m" XXX
 SG_ LONG_DIST : 64|11@1+ (0.1,0) [0|204.7] "m" XXX
 SG_ LAT_DIST : 75|12@1- (0.05,0) [-102.4|102.35] "m" XXX
 SG_ REL_SPEED : 87|11@1- (0.1,0) [-102.4|102.3] "m/s" XXX
""")
  return {"hyundai_canfd_radar_group3.dbc": "".join(parts)}
