# commands
uint8 SET_MODE=0
uint8 ESTOP=4
uint8 CLEAR_FAULT=5

# target modes
uint8 PASSIVE=0
uint8 STANDING=2
uint8 BALANCE=2
uint8 LOCOMOTION=3

uint8 command
# Target mode for SET_MODE. Ignored for ESTOP and CLEAR_FAULT.
uint8 target
