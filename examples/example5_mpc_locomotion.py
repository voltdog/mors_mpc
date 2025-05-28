import os
import sys
sys.path.insert(0, f"/home/{os.getlogin()}/lcm_msgs")

import lcm
from mors_msgs.robot_cmd_msg import robot_cmd_msg
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.std_int import std_int
from mors_msgs.gait_params_msg import gait_params_msg
import time
import numpy as np
import ForwardKinematics as fk
import TrajectoryGenerator  as tg
import BodyMovingControl as bm
import yaml

dt = 0.01

DURATION = 7.07

GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]
# GAIT_TYPE = [0.0, 0.25, 0.5, 0.75]
T_SW = 0.2 #0.2
T_ST = 0.2 #0.4
STRIDE_HEIGHT = 0.1

LIN_VEL_X = 0.15
LIN_VEL_Y = 0.0
ANG_VEL_Z = 0.0
POS_Z = 0.21
 
CONTROL_TYPE_CHANNEL = "CONTROL_TYPE"
ROBOT_CMD_CHANNEL = "ROBOT_CMD"#"ROBOT_REF"#
GAIT_PARAMS_CHANNEL = "GAIT_PARAMS"
SERVO_CMD_CHANNEL = "SERVO_CMD"

LEG_CONTROL = 1
SERVO_CONTROL = 2

X = 0
Y = 1
Z = 2
ROLL = 3
PITCH = 4
YAW = 5


def null_servo():
    global lc

    control_type_msg = std_int()
    control_type_msg.data = SERVO_CONTROL
    lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    # lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.torque = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.2]*12
    for i in range(10):
        lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
        time.sleep(0.01)

def go_robot_pos(cur_pos, ref_pos, tf):
    global cmd_msg, lc
    pos_traj, vel_traj = tg.create_multiple_trajectory(cur_pos, 
                                             ref_pos,
                                             tf, 
                                             dt)
    it = 0
    t = 0.0
    while it < len(pos_traj[0]):
        start = time.time()

        for i in range(6):
            cmd_msg.cmd_pose[i] = pos_traj[i][it]
            cmd_msg.cmd_vel[i] = vel_traj[i][it]
        lc.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())

        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
        it += 1
        t += dt

print("[Example5 MPC Locomotion]: starting...")

lc = lcm.LCM()
null_servo()

gait_prms_msg = gait_params_msg()
gait_prms_msg.gait_type = GAIT_TYPE
gait_prms_msg.t_st = T_ST
gait_prms_msg.t_sw = T_SW
gait_prms_msg.stride_height = STRIDE_HEIGHT
gait_prms_msg.standing = True
lc.publish(GAIT_PARAMS_CHANNEL, gait_prms_msg.encode())

control_type_msg = std_int()
control_type_msg.data = LEG_CONTROL
lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

cmd_msg = robot_cmd_msg()
cmd_msg.cmd_pose = [0.0]*6
cmd_msg.cmd_pose[Z] = 0.0
cmd_msg.cmd_vel = [0.0]*6
cmd_msg.active_legs = [True, True, True, True]
cmd_msg.adaptation_type = 0
lc.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())

t = 0.0
z_idle = POS_Z

cmd_vel = [0.0]*6
cmd_pose = [0.0]*6
# stand up
print("Standing up...")


try:
    # go_robot_pos([0.0, 0.0, 0.00,  #position
    #             0.0, 0.0, 0.0], #orientation,
    #             [0.0, 0.0, 0.0,  #position
    #             0.0, 0.0, 0.0], #orientation
    #             1.0)
    go_robot_pos([0.0, 0.0, 0.00,  #position
                0.0, 0.0, 0.0], #orientation,
                [0.0, 0.0, z_idle,  #position
                0.0, 0.0, 0.0], #orientation
                2.0)
    
    time.sleep(2)

    gait_prms_msg.standing = False#True# 
    # gait_prms_msg.t_sw = T_SW
    lc.publish(GAIT_PARAMS_CHANNEL, gait_prms_msg.encode())

    cmd_pose[Z] = z_idle
    cmd_msg.cmd_pose[Z] = z_idle
                        
    # cmd_msg.cmd_vel = cmd_vel[:]
    # lc.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())

    t = 0.0
    print("[Example5 MPC Locomotion]:  started")
    time.sleep(2)

    cnt = 0
    cmd_vel[X] = -LIN_VEL_X
    while t < DURATION:
        start = time.time()

        # if t > 3.15:
            # gait_prms_msg.t_sw = 0.15
            # gait_prms_msg.t_st = 0.2
            # gait_prms_msg.stride_height = 0.04
            
        # if t > 3.0:
        #     gait_prms_msg.gait_type = [0.0, 0.5, 0.5, 0.0]
        #     gait_prms_msg.t_st = 0.25
        #     gait_prms_msg.t_sw = 0.3

        # if t > 6.0:
        #     gait_prms_msg.gait_type = [0.0, 0.5, 0.5, 0.0]
        #     gait_prms_msg.t_st = 0.18
        #     gait_prms_msg.t_sw = 0.22

        # if t > 9.0:
        #     gait_prms_msg.gait_type = [0.0, 0.5, 0.5, 0.0]
        #     gait_prms_msg.t_st = 0.3
        #     gait_prms_msg.t_sw = 0.4
            
            # print("hey")

        # if cnt % 400 == 0:
        #     cmd_vel[X] = -cmd_vel[X]
        #     gait_prms_msg.standing = not gait_prms_msg.standing
        #     # gait_prms_msg.t_sw = 0.0
        #     lc.publish(GAIT_PARAMS_CHANNEL, gait_prms_msg.encode())

        cmd_vel[X] = LIN_VEL_X
        # cmd_pose[X] += cmd_vel[X] * dt
        cmd_vel[Y] = LIN_VEL_Y
        # cmd_pose[Y] += cmd_vel[Y] * dt
        cmd_vel[YAW] = ANG_VEL_Z
        # cmd_pose[YAW] += cmd_vel[YAW] * dt

        cmd_msg.cmd_pose = cmd_pose[:]
        cmd_msg.cmd_vel = cmd_vel[:]
        lc.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())
        lc.publish(GAIT_PARAMS_CHANNEL, gait_prms_msg.encode())

        t += dt
        cnt += 1
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
    
    cmd_vel[X] = 0.0
    cmd_vel[Y] = 0.0
    cmd_vel[YAW] = 0.0
    # cmd_pose[X] += cmd_vel[X] * dt

    cmd_msg.cmd_pose = cmd_pose[:]
    cmd_msg.cmd_vel = cmd_vel[:]
    lc.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())
    time.sleep(2)
    print("[Example5 MPC Locomotion]: Almost finish")

    gait_prms_msg.standing = True
    # gait_prms_msg.t_sw = 0.0
    lc.publish(GAIT_PARAMS_CHANNEL, gait_prms_msg.encode())
    time.sleep(1)
    # lay down
    go_robot_pos([cmd_pose[X], 0.0, z_idle,  #position
                  0.0, 0.0, 0.0], #orientation
                 [cmd_pose[X], 0.0, 0.015,  #position
                  0.0, 0.0, 0.0],
                 2.0)

    control_type_msg.data = SERVO_CONTROL
    lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.torque = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.8]*12
    for i in range(10):
        lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
        time.sleep(0.01)

    print("[Example5 MPC Locomotion]:  finished")
        
except KeyboardInterrupt:
    print("[Example5 MPC Locomotion]: Keyboard Interrupt")
    print("[Example5 MPC Locomotion]: Finishing process...")

    gait_prms_msg.standing = True
    # gait_prms_msg.t_sw = 0.0#T_SW
    lc.publish(GAIT_PARAMS_CHANNEL, gait_prms_msg.encode())
    # lay down
    go_robot_pos([0.0, 0.0, z_idle,  #position
                  0.0, 0.0, 0.0], #orientation
                 [0.0]*6,
                 2.0)

    control_type_msg.data = SERVO_CONTROL
    lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    # lc = lcm.LCM()
    # srv_cmd_msg = servo_cmd_msg()
    # srv_cmd_msg.velocity = [0.0]*12
    # srv_cmd_msg.torque = [0.0]*12
    # srv_cmd_msg.kp = [0.0]*12
    # srv_cmd_msg.kd = [0.3]*12
    # for i in range(10):
    #     lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
    #     time.sleep(0.01)
    null_servo()

    print("Process finished")


