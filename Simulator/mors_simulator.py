import time
from threading import Thread
import lcm

from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg 
from mors_msgs.imu_lcm_data import imu_lcm_data
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.contact_sensor_msg import contact_sensor_msg
from mors_msgs.odometry_msg import odometry_msg

from additional.mors_env import MorsMujocoEnv

import yaml
import numpy as np
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
print(f"Simulator base directory: {BASE_DIR}")

class Hardware_Level_Sim():
    def __init__(self):
        self.ref_joint_pos = [0]*12
        self.ref_joint_vel = [0]*12
        self.ref_joint_torq = [0]*12
        self.kp = [0]*12
        self.kd = [0]*12
        self.cur_joint_pos = [0]*12
        self.cur_joint_vel = [0]*12
        self.cur_joint_torq = [0]*12

        self.read_config()

        # init LCM thread for commands
        self.cmd_th = Thread(target=self.get_cmd, args=())
        self.cmd_th.daemon = True
        self.cmd_th.start()

        # init LCM for states
        self.servo_state_msg = servo_state_msg()
        self.lcm_imu_msg = imu_lcm_data()
        self.lcm_robot_state_msg = robot_state_msg()
        self.lcm_odom_msg = odometry_msg()
        self.lcm_contact_sensor_msg = contact_sensor_msg()

        self.lc_servo_state = lcm.LCM()
        self.lc_imu = lcm.LCM()
        self.lc_robot_state = lcm.LCM()
        self.lc_odom = lcm.LCM()
        self.lc_contact = lcm.LCM()

        self.lcm_imu_msg.orientation_covariance = [2.603e-07, 0.0, 0.0, 0.0, 2.603e-07, 0.0, 0.0, 0.0, 0.0]
        self.lcm_imu_msg.angular_velocity_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        self.lcm_imu_msg.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        
        self.sim_it = 0
        self.init_simulation()

        self.body_quaternion = [0.0, 0.0, 0.0, 1.0]
        self.body_lin_pos = [0.0]*3
        self.body_ang_vel = [0.0]*3
        self.body_lin_vel = np.array([0]*3, float)
        self.body_lin_acc = np.array([0]*3, float)
        self.force_dir = 1

        self.imu_data = [0]*13

        self.leg_data_transform = [ 1,  1,  1, 
                                    1,  1,  1,
                                    1,  1,  1,
                                    1,  1,  1]
        # self.leg_data_order = [6, 8, 7, 9, 11, 10, 0, 2, 1, 3, 5, 4]
        self.leg_data_order = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

    def read_config(self):
        with open(f"{BASE_DIR}/../config/simulation.yaml", "r") as f:
            sim_config = yaml.safe_load(f)

        self.sim_freq = sim_config.get("frequency", 500)
        self.mjcf_root = sim_config.get("mjcf_root", "MJCF/mors.xml")
        self.mjcf_root = f"{BASE_DIR}/{self.mjcf_root}"
        self.init_motor_angles = sim_config.get("init_motor_angles", [0.0, -1.57, 3.14,
                                                                      -0.0, 1.57, -3.14,
                                                                      -0.0, -1.57, 3.14,
                                                                      0.0, 1.57, -3.14])
        self.foot_contacts_enabled = sim_config.get("foot_contacts", True)
        self.full_state_enabled = sim_config.get("full_state", False)
        self.lcm_odometry_enabled = sim_config.get("odometry", True)
        self.foot_positions_type = sim_config.get("foot_positions_type", "local") # "global" or "local"

        self.external_disturbance_enabled = sim_config.get("external_disturbance", False)
        self.external_disturbance_value = sim_config.get("external_disturbance_value", 4000)
        self.external_disturbance_duration = sim_config.get("external_disturbance_duration", 0.02)
        self.external_disturbance_interval = sim_config.get("external_disturbance_interval", 2)

        self.joint_dir = sim_config.get("joint_dir", [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        self.joint_offset = sim_config.get("joint_offset", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.vel_max = sim_config.get("vel_max", [28.0, 28.0, 14.0])
        self.tau_max = sim_config.get("tau_max", [6.0, 6.0, 12.0])
        self.gear_ratio = sim_config.get("gear_ratio", 10.0)
        self.kt = sim_config.get("kt", 0.74)

        with open(f"{BASE_DIR}/../config/channels.yaml", "r") as f:
            lcm_config = yaml.safe_load(f)

        self.lcm_servo_cmd_channel = lcm_config.get("lcm_servo_cmd_channel", "SERVO_CMD")
        self.lcm_servo_state_channel = lcm_config.get("lcm_servo_state_channel", "SERVO_STATE")
        self.lcm_imu_channel = lcm_config.get("lcm_imu_channel", "IMU_DATA")
        self.lcm_odom_channel = lcm_config.get("lcm_odom_channel", "ODOMETRY")
        self.lcm_contact_sensor_channel = lcm_config.get("lcm_contact_sensor_channel", "CONTACT_SENSOR")
        self.lcm_robot_state_channel = lcm_config.get("lcm_robot_state_channel", "ROBOT_STATE")

        self.sim_period = 1.0/self.sim_freq
        self.first_step = True

    def init_simulation(self):
        self.env = MorsMujocoEnv(xml_path=self.mjcf_root,
                 sim_freq=self.sim_freq,
                 motor_kp=0.0,
                 motor_kd=0.0,
                 ext_disturbance_enabled=self.external_disturbance_enabled,
                #  motor_torque_limit=self.tau_max,
                 init_motor_angles=self.init_motor_angles)

        if self.external_disturbance_enabled:
            self.env.set_ext_forces_params(self.external_disturbance_value, 
                                        self.external_disturbance_duration, 
                                        self.external_disturbance_interval)
        
    def loop(self):
        step_start = time.time()
        self.sim_it += 1

        self.env.set_kpkd(self.kp, self.kd)
        self.env.step(self.ref_joint_pos, self.ref_joint_vel, self.ref_joint_torq)

        self.contact_flags = self.env.get_contact_flags()

        self.cur_joint_pos = self.env.get_motor_angles()
        self.cur_joint_vel = self.env.get_motor_velocities()
        self.cur_joint_torq = self.env.get_motor_torques()

        self.imu_data[0:3] = self.env.get_base_lin_acc()
        self.imu_data[3:7] = self.env.get_base_orientation()
        self.imu_data[7:10] = self.env.get_base_ang_vel()
        self.imu_data[10:] = self.env.get_base_orientation_euler()
        
        self.body_lin_pos = self.env.get_base_position()
        self.body_lin_vel = self.env.get_base_lin_vel()

        if self.foot_positions_type == "global":
            self.foot_pos = self.env.get_global_foot_positions()
        else:
            self.foot_pos = self.env.get_local_foot_positions()

        if self.first_step:
            self.first_step = False
            self.yaw_offset = self.imu_data[12]
        self.imu_data[12] -= self.yaw_offset
        
        self.__pub_servo_state(self.cur_joint_pos, self.cur_joint_vel, self.cur_joint_torq)
        self.__pub_imu_msg(self.imu_data)

        if self.foot_contacts_enabled:
            self.__pub_foot_contacts(self.contact_flags)
        
        if self.lcm_odometry_enabled:
            self.__pub_lcm_odom_msg(self.body_lin_pos, self.body_lin_vel, self.imu_data)

        if self.full_state_enabled:
            self.__pub_robot_state(imu_data=self.imu_data,
                                base_pos=self.body_lin_pos,
                                base_vel=self.body_lin_vel,
                                contact_states=self.contact_flags,
                                r1_pos=self.foot_pos[0],
                                l1_pos=self.foot_pos[1],
                                r2_pos=self.foot_pos[2],
                                l2_pos=self.foot_pos[3])
            
        elapsed = time.time() - step_start
        if elapsed < self.sim_period:
            time.sleep(self.sim_period - elapsed)

    def __pub_servo_state(self, joint_pos, joint_vel, joint_torq):
        for i in range(12):
            self.servo_state_msg.position[i] = self.joint_dir[i]*joint_pos[i] - self.joint_offset[i]
            self.servo_state_msg.velocity[i] = self.joint_dir[i]*joint_vel[i]
            self.servo_state_msg.torque[i] = self.joint_dir[i]*joint_torq[i] * self.gear_ratio / self.kt

        self.lc_servo_state.publish(self.lcm_servo_state_channel, self.servo_state_msg.encode())

    def __pub_imu_msg(self, imu_data : list):
        self.lcm_imu_msg.linear_acceleration = imu_data[:3]
        self.lcm_imu_msg.angular_velocity = imu_data[7:10]
        self.lcm_imu_msg.orientation_euler = imu_data[10:]
        self.lcm_imu_msg.orientation_quaternion = imu_data[3:7]
        
        self.lc_imu.publish(self.lcm_imu_channel, self.lcm_imu_msg.encode())

    def __pub_robot_state(self, imu_data, base_pos, base_vel, contact_states, r1_pos, l1_pos, r2_pos, l2_pos):
        for i in range(3):
            self.lcm_robot_state_msg.body.position[i] = base_pos[i]
            self.lcm_robot_state_msg.body.orientation[i] = imu_data[i+10]
            self.lcm_robot_state_msg.body.lin_vel[i] = base_vel[i]
            self.lcm_robot_state_msg.body.ang_vel[i] = imu_data[i+7]

            self.lcm_robot_state_msg.legs.r1_pos[i] = r1_pos[i]
            self.lcm_robot_state_msg.legs.l1_pos[i] = l1_pos[i]
            self.lcm_robot_state_msg.legs.r2_pos[i] = r2_pos[i]
            self.lcm_robot_state_msg.legs.l2_pos[i] = l2_pos[i]

        self.lcm_robot_state_msg.legs.contact_states = contact_states[:]
        # print(imu_data[11])
        self.lc_robot_state.publish(self.lcm_robot_state_channel, self.lcm_robot_state_msg.encode())

    def __pub_lcm_odom_msg(self, body_lin_pos, body_lin_vel, imu_data):
        self.lcm_odom_msg.position = body_lin_pos[:]
        self.lcm_odom_msg.orientation = imu_data[10:]
        self.lcm_odom_msg.lin_vel = body_lin_vel[:]
        self.lcm_odom_msg.ang_vel = imu_data[7:10]
        self.lc_odom.publish(self.lcm_odom_channel, self.lcm_odom_msg.encode())
    
    def __pub_foot_contacts(self, contact_flags):
        self.lcm_contact_sensor_msg.contact_states = contact_flags[:]
        self.lc_contact.publish(self.lcm_contact_sensor_channel, self.lcm_contact_sensor_msg.encode())


    def get_cmd(self):
        # init LCM
        lc = lcm.LCM()
        subscription = lc.subscribe(self.lcm_servo_cmd_channel, self.cmd_handler)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def cmd_handler(self, channel, data):
        msg = servo_cmd_msg.decode(data)
        # print(msg.kp)
        # print(f"Received message: {msg.position}")
        for i in range(12):
            self.ref_joint_pos[i] = self.joint_dir[i] * msg.position[i] - self.joint_offset[i]
            self.ref_joint_vel[i] = self.joint_dir[i] * msg.velocity[i]
            self.ref_joint_torq[i] = self.joint_dir[i] * msg.torque[i] * self.kt / self.gear_ratio
            self.kp[i] = msg.kp[i]
            self.kd[i] = msg.kd[i]

    def get_sim_period(self):
        return self.sim_period

def main(args=None):
    hw_lvl_sim = Hardware_Level_Sim()
    while True:
        hw_lvl_sim.loop()

if __name__ == '__main__':
    main()