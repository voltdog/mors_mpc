import os
import sys
sys.path.insert(0, f"/home/{os.getlogin()}/lcm_msgs")

import lcm
from mors_msgs.robot_cmd_msg import robot_cmd_msg
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.std_int import std_int
import time
import numpy as np
import ForwardKinematics as fk
import TrajectoryGenerator  as tg
import BodyMovingControl as bm
import yaml

dt = 0.01
 
CONTROL_TYPE_CHANNEL = "CONTROL_TYPE"
ROBOT_CMD_CHANNEL = "ROBOT_CMD"
ROBOT_STATE_CHANNEL = "ROBOT_STATE"
SERVO_CMD_CHANNEL = "SERVO_CMD"

SET_KP = 16
SET_KD = 1.3

LEG_CONTROL = 1
SERVO_CONTROL = 2

X = 0
Y = 1
Z = 2

def go_robot_pos(cur_pos, ref_pos, tf):
    global cmd_msg, lc_rbt_cmd
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
        lc_rbt_cmd.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())

        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
        it += 1
        t += dt

# def get_cur_state():
#     raw_angle_lst = [[],[],[],[],[],[],[],[],[],[],[],[]]
#     angle_lst = [0]*12

#     def lcm_handler(channel, data):
#         msg = servo_state_msg.decode(data)
#         for i in range(12):
#             raw_angle_lst[i].append(msg.position[i])

#     num = 50
#     lc = lcm.LCM()
#     subscription = lc.subscribe(SERVO_STATE_CHANNEL, lcm_handler)
#     for i in range(num):
#         lc.handle()
#         time.sleep(0.005)

#     for i in range(12):
#         angle_lst[i] = np.array(raw_angle_lst[i]).mean()

#     return angle_lst

print("Example4 starting...")

lc_ctrl_type = lcm.LCM()

control_type_msg = std_int()
control_type_msg.data = LEG_CONTROL
lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

lc_rbt_cmd = lcm.LCM()
cmd_msg = robot_cmd_msg()
cmd_msg.cmd_pose = [0.0]*6
cmd_msg.cmd_vel = [0.0]*6

lc_rbt_cmd.publish(ROBOT_CMD_CHANNEL, cmd_msg.encode())

# stand up
print("Standing up...")
z_idle = 0.2

try:
    go_robot_pos([0.0]*6,
                [0.0, 0.0, z_idle,  #position
                0.0, 0.0, 0.0], #orientation
                2.0)

    # time.sleep(5.0)


    it = 0

    x_dist = 0.0
    y_dist = 0.0
    z_dist = 0.2
    roll_dist = 0.0
    pitch_dist = 0.0
    yaw_dist = 0.0

    x_incr = 0.0008
    y_incr = 0.0008
    z_incr = 0.0008
    roll_incr = 0.004
    pitch_incr = 0.004
    yaw_incr = 0.004
    

    x_max = 0.11
    y_max = 0.09
    z_max = 0.24
    z_min = 0.1
    roll_max = 0.3
    pitch_max = 0.4
    yaw_max = 0.4

    lin_vel = 0.001
    ang_vel = 0.005

    x_vel = 0.0
    y_vel = 0.0
    z_vel = 0.0
    roll_vel = 0.0
    pitch_vel = 0.0
    yaw_vel = 0.0

    # p = [0.0]*12
    # p_new = [0.0]*12

    cnt = 0
    t = 0.0
    print("Example1 started")

    # go_robot_pos(
    #         [0.0, 0.0, z_idle,  #position
    #         0.0, 0.0, 0.0], #orientation
    #         [x_max, 0.0, z_idle,  #position
    #         0.0, 0.0, 0.0], #orientation
    #         1.0)
    # go_robot_pos(
    #     [x_max, 0.0, z_idle,  #position
    #      0.0, 0.0, 0.0], #orientation
    #     [-x_max, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     2.0)
    # go_robot_pos(
    #     [-x_max, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    
    # time.sleep(1.0)
    
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, y_max, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    # go_robot_pos(
    #     [0.0, y_max, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, -y_max, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     2.0)
    # go_robot_pos(
    #     [0.0, -y_max, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    
    # time.sleep(1.0)
    
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_max,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_max,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_min,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     2.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_min,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    
    # time.sleep(1.0)
    
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     roll_max, 0.0, 0.0], #orientation
    #     1.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     roll_max, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     -roll_max, 0.0, 0.0], #orientation
    #     2.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     -roll_max, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    
    # time.sleep(1.0)
    
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, pitch_max, 0.0], #orientation
    #     1.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, pitch_max, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, -pitch_max, 0.0], #orientation
    #     2.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, -pitch_max, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    
    # time.sleep(1.0)
    
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, yaw_max], #orientation
    #     1.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, yaw_max], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, -yaw_max], #orientation
    #     2.0)
    # go_robot_pos(
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, -yaw_max], #orientation
    #     [0.0, 0.0, z_idle,  #position
    #     0.0, 0.0, 0.0], #orientation
    #     1.0)
    
    time.sleep(1.0)
    # time.sleep(100)
    print("Almost finish")

    go_robot_pos([0.0, 0.0, z_idle,  #position
                  0.0, 0.0, 0.0], #orientation
                 [0.0]*6,
                 2.0)

    control_type_msg.data = SERVO_CONTROL
    lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.torque = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.8]*12
    for i in range(10):
        lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
        time.sleep(0.01)

    print("Example4 finished")
        
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    print("Finishing process...")

    control_type_msg.data = SERVO_CONTROL
    lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.torque = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.3]*12
    for i in range(10):
        lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
        time.sleep(0.01)

    print("Process finished")


