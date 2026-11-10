uint8 ACCEPTED=0
uint8 SUCCEEDED=1
uint8 REJECTED=2
uint8 BUSY=3
uint8 CANCELED=4
uint8 SUPERSEDED=5
uint8 TIMEOUT=6
uint8 FAILED=7
# UNKNOWN is local transport uncertainty, never an FSM TIMEOUT.
uint8 UNKNOWN=255
uint8 result
string reason
