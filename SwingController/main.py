import numpy as np
from threading import Thread
import yaml
import time
from transforms3d.euler import euler2mat
import argparse

from lcm_data_exchange_sc import LCMDataExchange
from swing_controller import SwingLegController

SWING = 0
STANCE = 1
LATE = 2

R1 = 0
L1 = 1
R2 = 2
L2 = 3

if __name__ == "__main__":
    print("SwingLegController starting...")

    parser = argparse.ArgumentParser(
                    prog='GaitScheduler',
                    description='Generates gaits for a quadruped robot',
                    epilog=' ')
    parser.add_argument('-c', '--config')  
    args = parser.parse_args()
    if args.config == None:
        config_address = "../config/"
    else:
        config_address = args.config
    if config_address[-1] != "/":
        config_address += "/"

    # read data from config
    # channel names
    with open(config_address + 'channels.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        gait_phase_channel = config_data['gait_phase']
        gait_params_channel = config_data['gait_params']
        robot_state_channel = config_data['robot_state']
        robot_cmd_channel = config_data['robot_cmd']
        foot_cmd_channel = config_data['foot_cmd']

    # module_dt
    with open(config_address + 'timesteps.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        module_dt = config_data['swing_controller_dt']

    # robot physical params
    with open(config_address + 'robot_config.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        bx = config_data['bx']
        by = config_data['by']
        l1 = config_data['l1']

    # swing leg controller params
    with open(config_address + 'swing_controller.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        interleave_x = config_data['interleave_x']
        interleave_y = config_data['interleave_y']
        dz_near_ground = config_data['dz_near_ground']
        k1_fsp = config_data['k1_fsp']
        k2_fsp = config_data['k2_fsp']

    # impedance control params
    with open(config_address + 'imp_config.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        Kp = config_data['Kp']
        Kd = config_data['Kd']

    # init lcm
    lcm_exch = LCMDataExchange(gait_params_cnannel = gait_params_channel,
                            gait_phase_channel = gait_phase_channel,
                            robot_state_channel = robot_state_channel)
    lcm_exch.set_kpkd(Kp, Kd)

    robot_state_th = Thread(target=lcm_exch.robot_state_thread, args=())
    robot_state_th.daemon = True
    robot_state_th.start()

    robot_cmd_th = Thread(target=lcm_exch.robot_cmd_thread, args=())
    robot_cmd_th.daemon = True
    robot_cmd_th.start()

    gait_params_th = Thread(target=lcm_exch.gait_params_thread, args=())
    gait_params_th.daemon = True
    gait_params_th.start()

    gait_phase_th = Thread(target=lcm_exch.gait_phase_thread, args=())
    gait_phase_th.daemon = True
    gait_phase_th.start()

    # init swing controller
    swing_controller = SwingLegController(timestep=module_dt,
                                          bx=bx,
                                          by=by,
                                          l1=l1,
                                          interleave_x=interleave_x,
                                          interleave_y=interleave_y,
                                          dz_near_ground=dz_near_ground,
                                          k1_fsp=k1_fsp,
                                          k2_fsp=k2_fsp,
                                          )

    t = 0.0

    print("SwingLegController started")

    while True:
        start = time.time()

        gait_phase, gait_phi = lcm_exch.get_gait_phase()
        t_st, t_sw, stride_height = lcm_exch.get_gait_params()
        ref_body_height, ref_body_yaw_vel, ref_body_vel = lcm_exch.get_robot_cmd()
        base_pos, base_orientation, base_lin_vel, base_ang_vel = lcm_exch.get_body_state()
        r1_pos, l1_pos, r2_pos, l2_pos = lcm_exch.get_legs_pos()
         
        # doing control
        R_body = euler2mat(base_orientation[0], base_orientation[1], base_orientation[2])

        foot_pos_global = np.array([np.array(base_pos) + R_body @ np.array(r1_pos), 
                                    np.array(base_pos) + R_body @ np.array(l1_pos), 
                                    np.array(base_pos) + R_body @ np.array(r2_pos), 
                                    np.array(base_pos) + R_body @ np.array(l2_pos)])
        
        swing_controller.set_gait_params(t_sw, t_st, stride_height)
        x, dx, ddx = swing_controller.step(phase_signal=gait_phase,
                                            phi_cur=gait_phi,
                                            ref_body_height=ref_body_height,
                                            ref_body_yaw_vel=ref_body_yaw_vel,
                                            ref_body_vel=ref_body_vel,
                                            base_pos=base_pos,
                                            base_lin_vel=base_lin_vel,
                                            base_rpy_rate=base_ang_vel,
                                            R_body=R_body,
                                            foot_pos_global=foot_pos_global)

        lcm_exch.send_foot_cmd(x, dx, ddx)
        
        t += module_dt
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > module_dt:
                # print(elapsed)
                break
