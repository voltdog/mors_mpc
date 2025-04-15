from threading import Thread
import yaml
import time
import argparse

from contact_state_fsm import *
from hopf_gait_generator import *
from lcm_data_exchange_gs import *
from command_shaper import CommandShaper
    

if __name__ == "__main__":
    print("GaitScheduler starting...")

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

    print(config_address)

    # read data from config
    # channel names
    with open(config_address + 'channels.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        gait_phase_channel = config_data['gait_phase']
        gait_params_channel = config_data['gait_params']
        robot_state_channel = config_data['robot_state']

    # module_dt
    with open(config_address + 'timesteps.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        module_dt = config_data['gait_scheduler_dt']

    # cpg params: ampl, alpha, lamb, a
    # fsm params: start_td_detecting
    with open(config_address + 'gait_scheduler.yaml', 'r') as file:
        config_data = yaml.safe_load(file)
        ampl = config_data['ampl']
        alpha = config_data['alpha']
        lamb = config_data['lamb']
        a = config_data['a']
        start_td_detecting = config_data['start_td_detecting']

    # init lcm
    standing = True
    pre_standing = True
    lcm_exch = LCMDataExchange(gait_params_cnannel = gait_params_channel,
                            gait_phase_channel = gait_phase_channel,
                            robot_state_channel = robot_state_channel)

    robot_state_th = Thread(target=lcm_exch.robot_state_thread, args=())
    robot_state_th.daemon = True
    robot_state_th.start()

    gait_params_th = Thread(target=lcm_exch.gait_params_thread, args=())
    gait_params_th.daemon = True
    gait_params_th.start()

    robot_ref_th = Thread(target=lcm_exch.robot_ref_thread, args=())
    robot_ref_th.daemon = True
    robot_ref_th.start()

    ref_gait, t_st, t_sw, standing = lcm_exch.get_gait_params()
    w_sw = np.pi/t_sw
    w_st = np.pi/t_st

    # init hopf oscillator based generator
    cpg = HopfGaitScheduler()
    cpg.set_static_params(ampl=ampl, alpha=alpha, lamb=lamb, a=a)
    cpg.set_gait_params(w_sw, w_st, ref_gait)
    cpg.init_integrator(module_dt)

    # init contact state descition making state machine
    contact_fsm = ContactStateFSM(start_td_detecting=start_td_detecting)

    # init command shaper
    cmd_shaper = CommandShaper(module_dt, 1.0)

    t = 0.0

    
    print("GaitScheduler started")

    while True:
        start = time.time()

        ref_gait, t_st, t_sw, standing = lcm_exch.get_gait_params()
        leg_contacts = lcm_exch.get_leg_contacts()
        foot_pos_local = lcm_exch.get_leg_pos_local()
        body_pos, body_orientation = lcm_exch.get_body_state()
        ref_pos, ref_orientation, ref_lin_vel, ref_ang_vel = lcm_exch.get_robot_ref()

        # gait scheduler
        w_sw = np.pi/t_sw
        w_st = np.pi/t_st

        cpg.set_gait_params(w_sw, w_st, ref_gait)
        phi_cur = cpg.step()

        if standing == True:
            phi_cur = [-0.5]*4
        elif standing == False and pre_standing == True:
            cpg.init_integrator(module_dt)

        phase_signal = contact_fsm.step(leg_contacts, phi_cur)

        # command shaper
        foot_pos_global = [np.array(body_pos) + foot_pos_local[0],
                           np.array(body_pos) + foot_pos_local[1],
                           np.array(body_pos) + foot_pos_local[2],
                           np.array(body_pos) + foot_pos_local[3]]
        
        x_ref, ref_body_vel_filtered, ref_body_yaw_vel_filtered = cmd_shaper.step(phase_signal=phase_signal,
                            foot_pos_global=foot_pos_global,
                            foot_pos_local=foot_pos_local,
                            ref_body_vel=ref_lin_vel,
                            ref_body_yaw_vel=ref_ang_vel[2],
                            ref_body_height=ref_pos[2])

        lcm_exch.send_gait_phase(phase=phase_signal,
                                 phi=phi_cur)
        lcm_exch.send_robot_cmd(orientation=x_ref[0:3], 
                                pos=x_ref[3:6],
                                ang_vel=x_ref[6:9],
                                lin_vel=x_ref[9:12])
        # phase_signal[1] = STANCE
        # phase_signal[2] = STANCE
        # phase_signal[0] = STANCE
        # phi_cur[1] = -0.5
        # phi_cur[2] = -0.5
        # phi_cur[0] = -0.5

        pre_standing = standing
        t += module_dt
        while(True):
            end = time.time()
            elapsed = end - start
            if elapsed > module_dt:
                # print(elapsed)
                break
