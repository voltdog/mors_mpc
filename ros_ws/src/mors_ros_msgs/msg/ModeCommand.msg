# commands
uint8 SET_MODE=0
uint8 START_ACTION=1
uint8 ESTOP=4
uint8 CLEAR_FAULT=5

# target modes
uint8 PASSIVE=0
uint8 STANDING=2
uint8 BALANCE=2
uint8 LOCOMOTION=3

# target actions
uint8 CHANGE_KINEMATICS=0

# CHANGE_KINEMATICS parameters
uint8 KNEE_INSIDE=0
uint8 KNEE_OUTSIDE=1
uint8 KNEE_BACK=2
uint8 KNEE_FRONT=3

uint8 command
# Target mode for SET_MODE or action for START_ACTION.
uint8 target
# Knee scheme for START_ACTION / CHANGE_KINEMATICS. Ignored otherwise.
uint8 params
