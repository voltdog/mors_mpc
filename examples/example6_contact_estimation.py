import os
import sys
sys.path.insert(0, f"/home/{os.getlogin()}/lcm_msgs")

import lcm
from mors_msgs.foot_cmd_msg import foot_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg
from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.std_int import std_int
from mors_msgs.phase_signal_msg import phase_signal_msg
import time
import numpy as np
import ForwardKinematics as fk
import TrajectoryGenerator  as tg
import yaml
from threading import Thread
import argparse

dt = 0.005

LEG_CMD_CHANNEL = "FOOT_CMD"
ROBOT_STATE_CHANNEL = "ROBOT_STATE"
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

R1 = 0
L1 = 1
R2 = 2
L2 = 3

parser = argparse.ArgumentParser(
                    prog='example1',
                    description='Test of leg contacts state estimation',
                    epilog='Have a nice day')
parser.add_argument('-l', '--leg')
args = parser.parse_args()
leg_num = int(args.leg)
print(args.leg)

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

contact_states = [False]*4

try:
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
except:
    raise RuntimeError("Cannot find imp_config.yaml")
leg_kp = config["Kp"]
leg_kd = config["Kd"]

def robot_state_handler(channel, data : robot_state_msg):
    global contact_states
    msg = robot_state_msg.decode(data)
    # print(f"{msg.contact_states[R2]} | {msg.r2_grf[2]:.2}")
    # print(f"{msg.l1_grf[2]:.2}")
    # if msg.contact_states[leg_num] == True:
    #     print("True")
    contact_states = msg.legs.contact_states[:]


def get_leg_state():
    # init LCM
    lc = lcm.LCM()
    subscription = lc.subscribe(ROBOT_STATE_CHANNEL, robot_state_handler)
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass


leg_state_th = Thread(target=get_leg_state, args=())
leg_state_th.daemon = True
leg_state_th.start()

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

phase_sig_msg = phase_signal_msg()

phase_sig_msg.phase = [STANCE]*4
phase_sig_msg.phase[leg_num] = SWING
phase_sig_msg.phi = [-0.5, -0.5, -0.5, -0.5]
lc.publish(GAIT_PHASE_CHANNEL, phase_sig_msg.encode())

control_type_msg = std_int()
control_type_msg.data = LEG_CONTROL
lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

t = 0.0
t_switch = 2.0

freq = 2*2*np.pi#12.56#6.0#
ampl = 0.06


r1_end_pos = [ l1+bx+0.02, -d1-d2-d3-by, -0.15]
# r1_end_pos = [-ampl*np.cos(freq*(t_switch + np.pi/2))+l1+bx, -d1-d2-d3-by, ampl*np.sin(freq*(t_switch + np.pi/2))-0.16]
l1_end_pos = [ l1+bx+0.03,  d1+d2+d3+by, -0.15]
r2_end_pos = [-l1-bx+0.02, -d1-d2-d3-by, -0.15]
l2_end_pos = [-l1-bx+0.02,  d1+d2+d3+by, -0.15]
# r1_end_pos = [-ampl*np.cos(freq*(t_switch + np.pi/2))+l1+bx, -d1-d2-d3-by, ampl*np.sin(freq*(t_switch + np.pi/2))-0.15]
# l1_end_pos = [-ampl*np.cos(freq*t_switch)+l1+bx,  d1+d2+d3+by, ampl*np.sin(freq*(t_switch))-0.15]
# r2_end_pos = [-ampl*np.cos(freq*t_switch)-l1-bx, -d1-d2-d3-by, ampl*np.sin(freq*(t_switch))-0.15]
# l2_end_pos = [-ampl*np.cos(freq*(t_switch + np.pi/2))-l1-bx,  d1+d2+d3+by, ampl*np.sin(freq*(t_switch + np.pi/2))-0.15]

r1_traj, _ = tg.create_multiple_trajectory(r1_cur_pos, r1_end_pos, t_switch, dt)
l1_traj, _ = tg.create_multiple_trajectory(l1_cur_pos, l1_end_pos, t_switch, dt)
r2_traj, _ = tg.create_multiple_trajectory(r2_cur_pos, r2_end_pos, t_switch, dt)
l2_traj, _ = tg.create_multiple_trajectory(l2_cur_pos, l2_end_pos, t_switch, dt)

it = 0

upper_limit = -0.22
lower_limit = -0.07
offset = 0.002
z = l1_end_pos[Z]
dz = offset/dt
down = False

print("Example1 started")
try:
    while(t < 9.2):
        start = time.time()
        if t < t_switch and it < len(l2_traj[X]):
            msg.r1_pos = [r1_traj[X][it], r1_traj[Y][it], r1_traj[Z][it]]
            msg.l1_pos = [l1_traj[X][it], l1_traj[Y][it], l1_traj[Z][it]]
            msg.r2_pos = [r2_traj[X][it], r2_traj[Y][it], r2_traj[Z][it]]
            msg.l2_pos = [l2_traj[X][it], l2_traj[Y][it], l2_traj[Z][it]]

            it += 1
        else:
            if down == True:
                if z > upper_limit:
                    z = z - offset
                    dz = -offset/dt
                else:
                    # trans = 1
                    down = False
                if contact_states[leg_num] == True:
                    down = False

            elif down == False:
                if z < lower_limit:
                    z = z + offset
                    dz = offset/dt
                else:
                    # trans = 1
                    down = True
            if leg_num == R1:
                msg.r1_pos[Z] = z
                msg.r1_vel[Z] = dz
            elif leg_num == L1:
                msg.l1_pos[Z] = z
                msg.l1_vel[Z] = dz
            elif leg_num == R2:
                msg.r2_pos[Z] = z
                msg.r2_vel[Z] = dz
            elif leg_num == L2:
                msg.l2_pos[Z] = z
                msg.l2_vel[Z] = dz
            # msg.r1_pos[Z] = ampl*np.sin(freq*(t + np.pi/2))-0.15
            # msg.r1_vel[Z] = ampl*freq*np.cos(freq*(t + np.pi/2))
            # msg.r1_acc[Z] = -ampl*freq*freq*np.sin(freq*(t + np.pi/2))

        # print(f"t: {t:.2f} | Kp: {msg.Kp[0]:.2f} | Kd: {msg.Kd[0]:.2f}")
        # print(f"t: {t:.2f} | {msg.l1_pos[Z]:.3f} | {msg.l1_vel[Z]}")
        # print(f"{msg.l1_pos[X]:.3f}")
        # print(f"{contact_states[L1]}")
        # print(f"{msg.l1_pos[Z]:.2f} | {dz}")
        
        lc.publish(LEG_CMD_CHANNEL, msg.encode())
        t += dt
        
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > dt:
                break


    msg.Kp = [0.0]*12
    msg.Kd = [0.0]*12
    msg.r1_pos[0] = l1+0.005+bx
    msg.r1_pos[1] = -d1-d2-d3-by
    msg.r1_pos[2] = -0.14
    msg.r1_vel = [0.0]*12
    lc.publish(LEG_CMD_CHANNEL, msg.encode())

    for i in range(10):
        time.sleep(0.01)

    control_type_msg.data = SERVO_CONTROL
    lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    phase_sig_msg.phase = [STANCE, STANCE, STANCE, STANCE]
    lc.publish(GAIT_PHASE_CHANNEL, phase_sig_msg.encode())

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
        
except KeyboardInterrupt:
    print("Keyboard Interrupt")
    print("Finishing process...")
    msg.imp_mode = [False]*4
    msg.grf_mode = [False]*4
    msg.Kp = [0.0]*12
    msg.Kd = [0.0]*12
    msg.r1_pos[0] = l1+0.005+bx
    msg.r1_pos[1] = -d1-d2-d3-by
    msg.r1_pos[2] = -0.14
    msg.r1_vel = [0.0]*12
    msg.r1_grf = [0.0]*12
    lc.publish(LEG_CMD_CHANNEL, msg.encode())

    control_type_msg.data = SERVO_CONTROL
    lc.publish(CONTROL_TYPE_CHANNEL, control_type_msg.encode())

    phase_sig_msg.phase = [STANCE, STANCE, STANCE, STANCE]
    lc.publish(GAIT_PHASE_CHANNEL, phase_sig_msg.encode())

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

