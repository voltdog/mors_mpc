# commands
uint8 SET_MODE=0
uint8 SET_ALGORITHM=3
uint8 ESTOP=4
uint8 CLEAR_FAULT=5
uint8 SET_EXTERNAL=6

# target modes
uint8 PASSIVE=0
uint8 BALANCE=2
uint8 LOCOMOTION=3

# target algorithms

uint8 command
# SET_MODE: mode; SET_ALGORITHM: stable algorithm ID (0..255).
# Ignored for SET_EXTERNAL, ESTOP and CLEAR_FAULT.
uint8 target
