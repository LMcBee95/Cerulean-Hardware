Packet Spec

0x12 = Header byte
Motor 1 = Signed int (1 byte) 
Motor 2 = Signed int (1 byte)
Motor 3 = Signed int (1 byte)
Motor 4 = Signed int (1 byte)
Motor 5 = Signed int (1 byte)
Motor 6 = Signed int (1 byte)
Motor 7 = Signed int (1 byte)
Motor 8 = Signed int (1 byte)
Tools 1 = 1 bit more each state of tool (maybe more if tool is complex)
Tools 2 = 1 bit more each state of tool (maybe more if tool is complex)
R = 1 byte
G = 1 byte
B = 1 byte
CRC 8 checksum (use 0xD5 as polynomial)
0x13 = Tail Byte


Motor signed bit representation
Use bit masking to get the sign bit, and then the motor value
Scale the percentage from 0 to 127 to 0% to 100%

Bit Values | Dec Value | Motor Percent 
-----------|-----------|---------------
0000 0000  |         0 |            0
0000 0001  |         1 |            1
0111 1111  |       127 |          100
1000 0000  |       128 |           -0
1000 0001  |       129 |           -1
1111 1111  |       255 |         -100


Tool Representation (Subject to change depending on tools)
Use bit masking to get off the values you need (magic!)

Bit # |   Tool   | State (0)  | State (1)
------|----------|------------|----------
  1   |    Claw  |       Open | Closed
  2   |Cam Mux 1 |      Cam 1 | Cam 2
  3   |Cam Mux 2 |      Cam 3 | Cam 4
  4   |    TBD   |        TBD | TBD
  5   |    TBD   |        TBD | TBD
  6   |    TBD   |        TBD | TBD
  7   |    TBD   |        TBD | TBD
  8   |    TBD   |        TBD | TBD
