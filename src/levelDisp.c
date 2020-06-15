#include "ch.h"
#include "hal.h"
#include "levelDisp.h"


/**
  level LCD display interface module via serial port
  connection: 9600,n,8,1,n
packet format:
[start byte][adr][level_code][dir][lift state 1][lift state 2][end byte]
start byte      : 0xfa
adr             : 0x0
level code      : 0x00 to 0xef, total 240 set, 0x83 = B1, 0x84 = 1
dir             : 0x0 to 0x6
lift state 1    : bitwise status
lift state 2    : bitwise status
end byte        : 0xfb

**/

#define LIFT_DIR_NONE   0x0
#define LIFT_DIR_UP     0x1
#define LIFT_DIR_DN     0x2
#define LIFT_RUNNING    0x4

#define LIFT_ST1_FIRE   (1 << 0)
#define LIFT_ST1_EQK    (1 << 1)
#define LIFT_ST1_FL     (1 << 2)
#define LIFT_ST1_PL     (1 << 3)
#define LIFT_ST1_OL     (1 << 4)
#define LIFT_ST1_PM     (1 << 5)
#define LIFT_ST1_OP     (1 << 6)

#define LIFT_ST2_LMT    (1 << 0)
#define LIFT_ST2_PK     (1 << 1)
#define LIFT_ST2_NOOP   (1 << 2)
#define LIFT_ST2_BL     (1 << 6)