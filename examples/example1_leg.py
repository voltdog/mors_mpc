import os
import sys
sys.path.insert(0, f"/home/{os.getlogin()}/lcm_msgs")

import lcm
from mors_msgs.foot_cmd_msg import foot_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg
from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.std_int import std_int
from mors_msgs.phase_signal_msg import phase_signal_msg
import time
import numpy as np
import ForwardKinematics as fk
import TrajectoryGenerator  as tg
import yaml

dt = 0.005

LEG_CMD_CHANNEL = "FOOT_CMD"
SERVO_STATE_CHANNEL = "SERVO_STATE"
SERVO_CMD_CHANNEL = "SERVO_CMD"
CONTROL_TYPE_CHANNEL = "CONTROL_TYPE"
GAIT_PHASE_CHANNEL = "GAIT_PHASE"

LEG_CONTROL = 1
SERVO_CONTROL = 2

SWING = 0
STANCE = 1

X = 0
Y = 1
Z = 2

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

print("Getting data from encoders...")
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
    time.sleep(0.01)

for i in range(12):
    angle_lst[i] = np.array(raw_angle_lst[i]).mean()

r1_cur_pos = fk.fkine_R1(angle_lst)
l1_cur_pos = fk.fkine_L1(angle_lst)
r2_cur_pos = fk.fkine_R2(angle_lst)
l2_cur_pos = fk.fkine_L2(angle_lst)

lc = lcm.LCM()
msg = foot_cmd_msg()

msg.r1_kp = leg_kp #[0.0]*3 #
msg.l1_kp = leg_kp
msg.r2_kp = leg_kp #[0.0]*3 #
msg.l2_kp = leg_kp #[0.0]*3 # 

msg.r1_kd = leg_kd #[0.0]*3 #
msg.l1_kd = leg_kd
msg.r2_kd = leg_kd #[0.0]*3 #leg_kd #
msg.l2_kd = leg_kd #[0.0]*3 #

msg.r1_pos = r1_cur_pos
msg.r1_vel = [0.0]*12
msg.l1_pos = l1_cur_pos
msg.l1_vel = [0.0]*12
msg.r2_pos = r2_cur_pos
msg.r2_vel = [0.0]*12
msg.l2_pos = l2_cur_pos
msg.l2_vel = [0.0]*12
lc.publish(LEG_CMD_CHANNEL, msg.encode())

t = 0.0
t_switch = 2.0

freq = 1*2*np.pi#12.56#6.0#
ampl = 0.04


r1_end_pos = [-ampl*np.cos(freq*(t_switch + np.pi/2))+l1+bx, -d1-d2-d3-by, ampl*np.sin(freq*(t_switch + np.pi/2))-0.15]
# r1_end_pos = [l1+bx, -d1-d2-d3-by, -0.15]
l1_end_pos = [-ampl*np.cos(freq*t_switch)+l1+bx,  d1+d2+d3+by, ampl*np.sin(freq*(t_switch))-0.15]
r2_end_pos = [-ampl*np.cos(freq*t_switch)-l1-bx, -d1-d2-d3-by, ampl*np.sin(freq*(t_switch))-0.15]
l2_end_pos = [-ampl*np.cos(freq*(t_switch + np.pi/2))-l1-bx,  d1+d2+d3+by, ampl*np.sin(freq*(t_switch + np.pi/2))-0.15]

r1_traj, _ = tg.create_multiple_trajectory(r1_cur_pos, r1_end_pos, t_switch, dt)
l1_traj, _ = tg.create_multiple_trajectory(l1_cur_pos, l1_end_pos, t_switch, dt)
r2_traj, _ = tg.create_multiple_trajectory(r2_cur_pos, r2_end_pos, t_switch, dt)
l2_traj, _ = tg.create_multiple_trajectory(l2_cur_pos, l2_end_pos, t_switch, dt)

it = 0

# set control type as leg_control
phase_sig_msg = phase_signal_msg()
phase_sig_msg.phase = [SWING, SWING, SWING, SWING]
phase_sig_msg.phi = [-0.5, -0.5, -0.5, -0.5]
lc.publish(GAIT_PHASE_CHANNEL, phase_sig_msg.encode())

lc_ctrl_type = lcm.LCM()

control_type_msg = std_int()
control_type_msg.data = LEG_CONTROL
lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

print("Example1 started")
try:
    while(t < 50.0):
        start = time.time()
        if t < t_switch and it < len(l2_traj[X]):
            msg.r1_pos = [r1_traj[X][it], r1_traj[Y][it], r1_traj[Z][it]]
            msg.l1_pos = [l1_traj[X][it], l1_traj[Y][it], l1_traj[Z][it]]
            msg.r2_pos = [r2_traj[X][it], r2_traj[Y][it], r2_traj[Z][it]]
            msg.l2_pos = [l2_traj[X][it], l2_traj[Y][it], l2_traj[Z][it]]

            it += 1
        # else:
        #     msg.r1_pos[X] = -ampl*np.cos(freq*(t + np.pi/2))+l1+bx
        #     msg.r1_vel[X] = ampl*freq*np.sin(freq*(t + np.pi/2))
        #     msg.r1_acc[X] = ampl*freq*freq*np.cos(freq*(t + np.pi/2))
        #     msg.r1_pos[Z] = ampl*np.sin(freq*(t + np.pi/2))-0.15
        #     msg.r1_vel[Z] = ampl*freq*np.cos(freq*(t + np.pi/2))
        #     msg.r1_acc[Z] = -ampl*freq*freq*np.sin(freq*(t + np.pi/2))

        #     msg.l1_pos[X] = -ampl*np.cos(freq*t)+l1+bx
        #     msg.l1_vel[X] = ampl*freq*np.sin(freq*t)
        #     msg.l1_acc[X] = ampl*freq*freq*np.cos(freq*t)
        #     msg.l1_pos[Z] = ampl*np.sin(freq*(t))-0.15
        #     msg.l1_vel[Z] = ampl*freq*np.cos(freq*(t))
        #     msg.l1_acc[Z] = -ampl*freq*freq*np.sin(freq*(t))

        #     msg.r2_pos[X] = -ampl*np.cos(freq*t)-l1-bx
        #     msg.r2_vel[X] = ampl*freq*np.sin(freq*t)
        #     msg.r2_acc[X] = ampl*freq*freq*np.cos(freq*t)
        #     msg.r2_pos[Z] = ampl*np.sin(freq*(t))-0.15
        #     msg.r2_vel[Z] = ampl*freq*np.cos(freq*(t))
        #     msg.r2_acc[Z] = -ampl*freq*freq*np.sin(freq*(t))

        #     msg.l2_pos[X] = -ampl*np.cos(freq*(t + np.pi/2))-l1-bx
        #     msg.l2_vel[X] = ampl*freq*np.sin(freq*(t + np.pi/2))
        #     msg.l2_acc[X] = ampl*freq*freq*np.cos(freq*(t + np.pi/2))
        #     msg.l2_pos[Z] = ampl*np.sin(freq*(t + np.pi/2))-0.15
        #     msg.l2_vel[Z] = ampl*freq*np.cos(freq*(t + np.pi/2))
        #     msg.l2_acc[Z] = -ampl*freq*freq*np.sin(freq*(t + np.pi/2))

        # print(f"t: {t:.2f} | Kp: {msg.Kp[0]:.2f} | Kd: {msg.Kd[0]:.2f}")
        # print(f"t: {t:.2f} | {msg.r1_pos[X]:.3f} | {msg.r1_vel[X]}")
        
        lc.publish(LEG_CMD_CHANNEL, msg.encode())
        t += dt
        
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break
    print("Almost finish")

    msg.r1_kp = [0.0]*3
    msg.l1_kp = [0.0]*3
    msg.r2_kp = [0.0]*3
    msg.l2_kp = [0.0]*3
    msg.r1_kd = [0.0]*3
    msg.l1_kd = [0.0]*3
    msg.r2_kd = [0.0]*3
    msg.l2_kd = [0.0]*3
    msg.r1_pos[0] = l1+0.005+bx
    msg.r1_pos[1] = -d1-d2-d3-by
    msg.r1_pos[2] = -0.14
    msg.r1_vel = [0.0]*12
    lc.publish(LEG_CMD_CHANNEL, msg.encode())

    control_type_msg.data = SERVO_CONTROL
    lc_ctrl_type.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    phase_sig_msg.phase = [STANCE, STANCE, STANCE, STANCE]
    lc.publish(GAIT_PHASE_CHANNEL, phase_sig_msg.encode())

    for i in range(2):
        time.sleep(0.01)
    print("Set zero leg command")

    # for i in range(10):
    #     lc = lcm.LCM()
    #     srv_cmd_msg = servo_cmd_msg()
    #     srv_cmd_msg.velocity = [0.0]*12
    #     srv_cmd_msg.kp = [0.0]*12
    #     srv_cmd_msg.kd = [0.6]*12
    #     lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    print("Set zero servo command")

    for i in range(10):
        time.sleep(0.15)

    # srv_cmd_msg.kd = [0.0]*12
    # lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    print("Example1 finished")
        
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    print("Finishing process...")

    msg.r1_kp = [0.0]*3
    msg.l1_kp = [0.0]*3
    msg.r2_kp = [0.0]*3
    msg.l2_kp = [0.0]*3
    msg.r1_kd = [0.0]*3
    msg.l1_kd = [0.0]*3
    msg.r2_kd = [0.0]*3
    msg.l2_kd = [0.0]*3
    msg.r1_pos[0] = l1+0.005+bx
    msg.r1_pos[1] = -d1-d2-d3-by
    msg.r1_pos[2] = -0.14
    msg.r1_vel = [0.0]*12
    lc.publish(LEG_CMD_CHANNEL, msg.encode())

    lc = lcm.LCM()
    srv_cmd_msg = servo_cmd_msg()
    srv_cmd_msg.velocity = [0.0]*12
    srv_cmd_msg.kp = [0.0]*12
    srv_cmd_msg.kd = [0.6]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

    for i in range(10):
        time.sleep(0.15)

    srv_cmd_msg.kd = [0.0]*12
    lc.publish(SERVO_CMD_CHANNEL, srv_cmd_msg.encode())

