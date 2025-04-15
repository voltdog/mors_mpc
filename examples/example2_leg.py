import os
# import sys
# sys.path.insert(0, f"/home/{os.getlogin()}/lcm_msgs")

import lcm
from mors_msgs.grf_cmd_msg import grf_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg
from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.std_int import std_int
import time
import numpy as np
import ForwardKinematics as fk
import TrajectoryGenerator  as tg

dt = 0.005

GRF_CMD_CHANNEL = "GRF_CMD"
SERVO_STATE_CHANNEL = "SERVO_STATE"
SERVO_CMD_CHANNEL = "SERVO_CMD"

CONTROL_TYPE_CHANNEL = "CONTROL_TYPE"

LEG_CONTROL = 1
SERVO_CONTROL = 2

X = 0
Y = 1
Z = 2

# lcm data
lc = lcm.LCM()
msg = grf_cmd_msg()

msg.r1_grf = [0.0]*12
msg.l1_grf = [0.0]*12
msg.r2_grf = [0.0]*12
msg.l2_grf = [0.0]*12
lc.publish(GRF_CMD_CHANNEL, msg.encode())

srv_cmd_msg = servo_cmd_msg()
srv_cmd_msg.velocity = [0.0]*12
srv_cmd_msg.kp = [0.0]*12
srv_cmd_msg.kd = [0.0]*12
lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

# set control type as leg_control
lc_ctrl_type = lcm.LCM()

control_type_msg = std_int()
control_type_msg.data = LEG_CONTROL
lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

t = 0.0

print("Example1 started")
try:
    while(t < 30.0):
        start = time.time()

        msg.r1_grf[Z] = -20.0
        msg.l1_grf[Z] = -20.0
        msg.r2_grf[Z] = -20.0
        msg.l2_grf[Z] = -20.0

        # print(f"t: {t:.2f} | Kp: {msg.Kp[0]:.2f} | Kd: {msg.Kd[0]:.2f}")
        # print(f"t: {t:.2f} | {msg.r1_pos[X]:.3f} | {msg.r1_vel[X]}")
        
        lc.publish(GRF_CMD_CHANNEL, msg.encode())
        t += dt
        
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
        

    msg.r1_grf = [0.0]*3
    msg.l1_grf = [0.0]*3
    msg.r2_grf = [0.0]*3
    msg.l2_grf = [0.0]*3
    lc.publish(GRF_CMD_CHANNEL, msg.encode())

    for i in range(10):
        time.sleep(0.01)

    lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [1.2]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    for i in range(10):
        time.sleep(0.3)

    srv_cmd_msg.kd = [0.0]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    print("Example1 finished")
        
except KeyboardInterrupt:
    msg.r1_grf = [0.0]*3
    msg.l1_grf = [0.0]*3
    msg.r2_grf = [0.0]*3
    msg.l2_grf = [0.0]*3
    lc.publish(GRF_CMD_CHANNEL, msg.encode())

    lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.3]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    for i in range(10):
        time.sleep(0.15)

    srv_cmd_msg.kd = [0.0]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

