import os
import sys
sys.path.insert(0, f"/home/{os.getlogin()}/lcm_msgs")

import lcm
from mors_msgs.foot_cmd_msg import foot_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg
from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.std_int import std_int
import time
import numpy as np
import ForwardKinematics as fk
import TrajectoryGenerator  as tg
import BodyMovingControl as bm
import yaml

dt = 0.005
 
LEG_CMD_CHANNEL = "FOOT_CMD"
SERVO_STATE_CHANNEL = "SERVO_STATE"
SERVO_CMD_CHANNEL = "SERVO_CMD"
CONTROL_TYPE_CHANNEL = "CONTROL_TYPE"

SET_KP = 16
SET_KD = 1.3

LEG_CONTROL = 1
SERVO_CONTROL = 2

X = 0
Y = 1
Z = 2

def go_servo_pos(cur_angles, ref_angles, tf):
    global srv_cmd_msg, lc_srv_cmd
    go_null_traj = tg.create_multiple_trajectory(cur_angles, 
                                             ref_angles,
                                             tf, 
                                             dt)
    it = 0
    t = 0.0
    while it < len(go_null_traj[0]):
        start = time.time()

        for i in range(12):
            srv_cmd_msg.position[i] = go_null_traj[i][it]
        lc_srv_cmd.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
        it += 1
        t += dt

def get_cur_angles():
    raw_angle_lst = [[],[],[],[],[],[],[],[],[],[],[],[]]
    angle_lst = [0]*12

    def lcm_handler(channel, data):
        msg = servo_state_msg.decode(data)
        for i in range(12):
            raw_angle_lst[i].append(msg.position[i])

    num = 50
    lc = lcm.LCM()
    subscription = lc.subscribe(SERVO_STATE_CHANNEL, lcm_handler)
    for i in range(num):
        lc.handle()
        time.sleep(0.005)

    for i in range(12):
        angle_lst[i] = np.array(raw_angle_lst[i]).mean()

    return angle_lst

# read config
config = {}
current_working_directory = os.path.dirname(__file__)
config_path = current_working_directory+'/../config/robot_config.yaml'
try:
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
except:
    raise RuntimeError("Cannot find robot_config.yaml")
bx = config["bx"]
by = config["by"]
l1 = config["l1"]
d1 = config["d1"]
d2 = config["d2"]
d3 = config["d3"]

config_path = current_working_directory+'/../config/imp_config.yaml'
try:
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
except:
    raise RuntimeError("Cannot find imp_config.yaml")
leg_kp = config["Kp"]
leg_kd = config["Kd"]

body_moving = bm.BodyMovingControl()

print("Getting data from encoders...")
angle_lst = get_cur_angles()

lc_ctrl_type = lcm.LCM()

control_type_msg = std_int()
control_type_msg.data = SERVO_CONTROL
lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

lc_srv_cmd = lcm.LCM()
srv_cmd_msg = servo_cmd_msg()
srv_cmd_msg.position = angle_lst[:]
srv_cmd_msg.velocity = [0.0]*12
srv_cmd_msg.torque = [0.0]*12
srv_cmd_msg.kp = [0.0]*12
srv_cmd_msg.kd = [SET_KD]*12
lc_srv_cmd.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

# stand up
print("Standing up...")
kp = 0.0
while True:
    if kp < SET_KP:
        kp += 0.01
        if kp >= SET_KP/6:
            kp += 0.06
        srv_cmd_msg.kp = [kp]*12
        lc_srv_cmd.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
        time.sleep(0.002)
    else:
        break


go_servo_pos(angle_lst,
             [0.0, -np.pi/2,  np.pi,
              0.0,  np.pi/2, -np.pi,
              0.0, -np.pi/2,  np.pi,
              0.0,  np.pi/2, -np.pi], 
              2.0)
go_servo_pos([0.0, -np.pi/2,  np.pi,
              0.0,  np.pi/2, -np.pi,
              0.0, -np.pi/2,  np.pi,
              0.0,  np.pi/2, -np.pi], 
             [0.0, -np.pi/2,  2.6,
              0.0,  np.pi/2, -2.6,
              0.0, -np.pi/2,  2.6,
              0.0,  np.pi/2, -2.6], 
              1.0)


angle_lst = get_cur_angles()

r1_cur_pos = fk.fkine_R1(angle_lst)
l1_cur_pos = fk.fkine_L1(angle_lst)
r2_cur_pos = fk.fkine_R2(angle_lst)
l2_cur_pos = fk.fkine_L2(angle_lst)

lc = lcm.LCM()
msg = foot_cmd_msg()

msg.Kp = leg_kp*4
msg.Kd = leg_kd*4
msg.r1_pos = r1_cur_pos
msg.r1_vel = [0.0]*12
msg.l1_pos = l1_cur_pos
msg.l1_vel = [0.0]*12
msg.r2_pos = r2_cur_pos
msg.r2_vel = [0.0]*12
msg.l2_pos = l2_cur_pos
msg.l2_vel = [0.0]*12

lc.publish(LEG_CMD_CHANNEL, msg.encode())

srv_cmd_msg.kp = [0.0]*12
srv_cmd_msg.kd = [0.0]*12
lc_srv_cmd.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

control_type_msg.data = LEG_CONTROL
lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

t = 0.0
t_switch = 2.0

r1_end_pos = [r1_cur_pos[X], r1_cur_pos[Y], r1_cur_pos[Z]-0.15]
l1_end_pos = [l1_cur_pos[X], l1_cur_pos[Y], l1_cur_pos[Z]-0.15]
r2_end_pos = [r2_cur_pos[X], r2_cur_pos[Y], r2_cur_pos[Z]-0.15]
l2_end_pos = [l2_cur_pos[X], l2_cur_pos[Y], l2_cur_pos[Z]-0.15]

r1_traj = tg.create_multiple_trajectory(r1_cur_pos, r1_end_pos, t_switch, dt)
l1_traj = tg.create_multiple_trajectory(l1_cur_pos, l1_end_pos, t_switch, dt)
r2_traj = tg.create_multiple_trajectory(r2_cur_pos, r2_end_pos, t_switch, dt)
l2_traj = tg.create_multiple_trajectory(l2_cur_pos, l2_end_pos, t_switch, dt)

it = 0

x_dist = 0.0
x_incr = 0.0001
x_max = 0.06

y_dist = 0.0
y_incr = 0.0001
y_max = 0.04

z_dist = 0.0
z_incr = 0.0001
z_max = 0.04

roll_dist = 0.0
roll_incr = 0.0005
roll_max = 0.2

pitch_dist = 0.0
pitch_incr = 0.0005
pitch_max = 0.2

yaw_dist = 0.0
yaw_incr = 0.0005
yaw_max = 0.2

p = [0.0]*12
p_new = [0.0]*12

cnt = 0
print("Example1 started")
try:
    while(cnt < 5):
        start = time.time()
        if t < t_switch and it < len(l2_traj[X]):
            msg.r1_pos = [r1_traj[X][it], r1_traj[Y][it], r1_traj[Z][it]]
            msg.l1_pos = [l1_traj[X][it], l1_traj[Y][it], l1_traj[Z][it]]
            msg.r2_pos = [r2_traj[X][it], r2_traj[Y][it], r2_traj[Z][it]]
            msg.l2_pos = [l2_traj[X][it], l2_traj[Y][it], l2_traj[Z][it]]

            it += 1
        else:
            # x dir
            if 2.0 < t < 14.0:
                if x_dist >= x_max:
                    x_incr = -0.0001
                elif x_dist <= -x_max:
                    x_incr = 0.0001
                x_dist += x_incr
                msg.r1_pos[X] = r1_end_pos[X] + x_dist
                msg.l1_pos[X] = l1_end_pos[X] + x_dist
                msg.r2_pos[X] = r2_end_pos[X] + x_dist
                msg.l2_pos[X] = l2_end_pos[X] + x_dist
            elif 14.0 < t < 22.0:
                if y_dist >= y_max:
                    y_incr = -0.0001
                elif y_dist <= -y_max:
                    y_incr = 0.0001
                y_dist += y_incr
                msg.r1_pos[Y] = r1_end_pos[Y] + y_dist
                msg.l1_pos[Y] = l1_end_pos[Y] + y_dist
                msg.r2_pos[Y] = r2_end_pos[Y] + y_dist
                msg.l2_pos[Y] = l2_end_pos[Y] + y_dist
            elif 22.0 < t < 30.0:
                if z_dist >= z_max:
                    z_incr = -0.0001
                elif z_dist <= -z_max:
                    z_incr = 0.0001
                z_dist += z_incr
                msg.r1_pos[Z] = r1_end_pos[Z] + z_dist
                msg.l1_pos[Z] = l1_end_pos[Z] + z_dist
                msg.r2_pos[Z] = r2_end_pos[Z] + z_dist
                msg.l2_pos[Z] = l2_end_pos[Z] + z_dist

            elif 30.0 < t < 38.0:
                if roll_dist >= roll_max:
                    roll_incr = -0.0005
                elif roll_dist <= -roll_max:
                    roll_incr = 0.0005
                roll_dist += roll_incr

                msg.r1_pos[Y] = r1_end_pos[Y] * np.cos(roll_dist) - r1_end_pos[Z] * np.sin(roll_dist)
                msg.r1_pos[Z] = r1_end_pos[Y] * np.sin(roll_dist) + r1_end_pos[Z] * np.cos(roll_dist)
                msg.l1_pos[Y] = l1_end_pos[Y] * np.cos(roll_dist) - l1_end_pos[Z] * np.sin(roll_dist)
                msg.l1_pos[Z] = l1_end_pos[Y] * np.sin(roll_dist) + l1_end_pos[Z] * np.cos(roll_dist)
                msg.r2_pos[Y] = r2_end_pos[Y] * np.cos(roll_dist) - r2_end_pos[Z] * np.sin(roll_dist)
                msg.r2_pos[Z] = r2_end_pos[Y] * np.sin(roll_dist) + r2_end_pos[Z] * np.cos(roll_dist)
                msg.l2_pos[Y] = l2_end_pos[Y] * np.cos(roll_dist) - l2_end_pos[Z] * np.sin(roll_dist)
                msg.l2_pos[Z] = l2_end_pos[Y] * np.sin(roll_dist) + l2_end_pos[Z] * np.cos(roll_dist)
            elif 38.0 < t < 46.0:
                if pitch_dist >= pitch_max:
                    pitch_incr = -0.0005
                elif pitch_dist <= -pitch_max:
                    pitch_incr = 0.0005
                pitch_dist += pitch_incr

                msg.r1_pos[X] =  r1_end_pos[X] * np.cos(pitch_dist) + r1_end_pos[Z] * np.sin(pitch_dist)
                msg.r1_pos[Z] = -r1_end_pos[X] * np.sin(pitch_dist) + r1_end_pos[Z] * np.cos(pitch_dist)
                msg.l1_pos[X] =  l1_end_pos[X] * np.cos(pitch_dist) + l1_end_pos[Z] * np.sin(pitch_dist)
                msg.l1_pos[Z] = -l1_end_pos[X] * np.sin(pitch_dist) + l1_end_pos[Z] * np.cos(pitch_dist)
                msg.r2_pos[X] =  r2_end_pos[X] * np.cos(pitch_dist) + r2_end_pos[Z] * np.sin(pitch_dist)
                msg.r2_pos[Z] = -r2_end_pos[X] * np.sin(pitch_dist) + r2_end_pos[Z] * np.cos(pitch_dist)
                msg.l2_pos[X] =  l2_end_pos[X] * np.cos(pitch_dist) + l2_end_pos[Z] * np.sin(pitch_dist)
                msg.l2_pos[Z] = -l2_end_pos[X] * np.sin(pitch_dist) + l2_end_pos[Z] * np.cos(pitch_dist)
            elif 46.0 < t < 54.0:
                if yaw_dist >= yaw_max:
                    yaw_incr = -0.0005
                elif yaw_dist <= -yaw_max:
                    yaw_incr = 0.0005
                yaw_dist += yaw_incr

                msg.r1_pos[X] =  r1_end_pos[X] * np.cos(yaw_dist) - r1_end_pos[Y] * np.sin(yaw_dist)
                msg.r1_pos[Y] =  r1_end_pos[X] * np.sin(yaw_dist) + r1_end_pos[Y] * np.cos(yaw_dist)
                msg.l1_pos[X] =  l1_end_pos[X] * np.cos(yaw_dist) - l1_end_pos[Y] * np.sin(yaw_dist)
                msg.l1_pos[Y] =  l1_end_pos[X] * np.sin(yaw_dist) + l1_end_pos[Y] * np.cos(yaw_dist)
                msg.r2_pos[X] =  r2_end_pos[X] * np.cos(yaw_dist) - r2_end_pos[Y] * np.sin(yaw_dist)
                msg.r2_pos[Y] =  r2_end_pos[X] * np.sin(yaw_dist) + r2_end_pos[Y] * np.cos(yaw_dist)
                msg.l2_pos[X] =  l2_end_pos[X] * np.cos(yaw_dist) - l2_end_pos[Y] * np.sin(yaw_dist)
                msg.l2_pos[Y] =  l2_end_pos[X] * np.sin(yaw_dist) + l2_end_pos[Y] * np.cos(yaw_dist)

            elif t >= 54.0:
                t %= 2.0
                cnt += 1
                time.sleep(5.0)
                print(f"cnt={cnt}")

        lc.publish(LEG_CMD_CHANNEL, msg.encode())
        t += dt
        
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
    print("Almost finish")

    angle_lst = get_cur_angles()
    control_type_msg.data = SERVO_CONTROL
    lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())
    srv_cmd_msg.kp = [SET_KP]*12
    srv_cmd_msg.kd = [SET_KD]*12
    
    go_servo_pos(angle_lst,
             [0.0, -np.pi/2,  np.pi,
              0.0,  np.pi/2, -np.pi,
              0.0, -np.pi/2,  np.pi,
              0.0,  np.pi/2, -np.pi], 
              3.0)

    msg.Kp = [0.0]*12
    msg.Kd = [0.0]*12
    msg.r1_pos[0] = l1+0.005+bx
    msg.r1_pos[1] = -d1-d2-d3-by
    msg.r1_pos[2] = -0.14
    msg.r1_vel = [0.0]*12
    lc.publish(LEG_CMD_CHANNEL, msg.encode())

    for i in range(2):
        time.sleep(0.01)
    print("Set zero leg command")

    lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.3]*12
    for i in range(10):
        lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())
        time.sleep(0.01)

    print("Set zero servo command")

    for i in range(10):
        time.sleep(0.15)

    srv_cmd_msg.kd = [0.0]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    print("Example1 finished")
        
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    print("Finishing process...")

    msg.Kp = [0.0]*12
    msg.Kd = [0.0]*12
    msg.r1_pos[0] = l1+0.005+bx
    msg.r1_pos[1] = -d1-d2-d3-by
    msg.r1_pos[2] = -0.14
    msg.r1_vel = [0.0]*12
    lc.publish(LEG_CMD_CHANNEL, msg.encode())

    control_type_msg.data = SERVO_CONTROL
    lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

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

