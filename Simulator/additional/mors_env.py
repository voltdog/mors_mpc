"""
MuJoCo implementation of MORS Gym environment.
"""

import os
import inspect
import time
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation
from transforms3d.quaternions import quat2mat

from additional.motor_accurate import MotorAccurate


NUM_SUBSTEPS = 2
NUM_MOTORS = 12

MOTOR_NAMES = [
    "abad_joint_R1", "hip_joint_R1", "knee_joint_R1",
    "abad_joint_L1", "hip_joint_L1", "knee_joint_L1",
    "abad_joint_R2", "hip_joint_R2", "knee_joint_R2",
    "abad_joint_L2", "hip_joint_L2", "knee_joint_L2",
]

MOTOR_VELOCITY_MAX = [28, 28, 14]
MOTOR_TORQUE_MAX = [6.0, 6.0, 12.0]
BASE_ORIENTATION_MAX = 1.57

class MorsMujocoEnv():

    def __init__(self,
                 xml_path="../MJCF/mors.xml",
                 sim_freq=500,
                 motor_kp=30.0,
                 motor_kd=0.1,
                 ext_disturbance_enabled=False,
                #  motor_torque_limit=MOTOR_TORQUE_MAX,
                #  motor_velocity_limit=MOTOR_VELOCITY_MAX,
                 init_motor_angles=[ 0.0, -1.57,  3.14,
                                    -0.0,  1.57, -3.14,
                                    -0.0, -1.57,  3.14,
                                     0.0,  1.57, -3.14]):
        
        self._motor_kp = motor_kp
        self._motor_kd = motor_kd
        # self._motor_torque_limit = motor_torque_limit
        self._xml_path = xml_path
        self._sim_freq = sim_freq
        self._ext_disturbance_enabled = ext_disturbance_enabled
        self._time_step = 1 / sim_freq
        self._env_step_counter = 0

        # ---------------- MuJoCo ----------------
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self._time_step
        self.viewer = mujoco.viewer.launch_passive(self.model, 
                                                   self.data,
                                                   show_left_ui=False,
                                                   show_right_ui=False)
        self.data.qpos[7:7+NUM_MOTORS] = init_motor_angles[:]
        mujoco.mj_forward(self.model, self.data)

        # motor id mapping
        self._motor_id_list = [mujoco.mj_name2id(self.model,
                                                 mujoco.mjtObj.mjOBJ_JOINT,
                                                 name)
                               for name in MOTOR_NAMES]
        
        # --------------- Init motors ---------------
        self._motor_model = MotorAccurate(NUM_MOTORS)
        self._motor_model.set_kp(self._motor_kp)
        self._motor_model.set_kd(self._motor_kd)

        # --------------- External disturbance ---------------
        self._force_dir = 1

      
    def step(self, 
             ref_joint_angle : list[float], 
             ref_joint_velocity : list[float], 
             ref_joint_torque : list[float]):
        if len(ref_joint_angle) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor angles, got {len(ref_joint_angle)}")
        if len(ref_joint_velocity) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor velocities, got {len(ref_joint_velocity)}")
        if len(ref_joint_torque) != NUM_MOTORS:
            raise ValueError(f"Expected {NUM_MOTORS} motor torques, got {len(ref_joint_torque)}")
        self._env_step_counter += 1

        if self._ext_disturbance_enabled:
            self._apply_force()

        self.__apply_action(ref_joint_angle, ref_joint_velocity, ref_joint_torque)
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        
    def __apply_action(self, ref_joint_angle, ref_joint_velocity, ref_joint_torque):

        motor_angles = self.get_motor_angles()
        motor_velocities = self.get_motor_velocities()

        self._motor_model.set_sensor_data(motor_angles, motor_velocities)
        self._motor_model.set_ref_angle(ref_joint_angle)
        self._motor_model.set_ref_vel(ref_joint_velocity)
        self._motor_model.set_ref_torque(ref_joint_torque)
        self._motor_model.set_kp(self._motor_kp)
        self._motor_model.set_kd(self._motor_kd)
        torque = self._motor_model.step()

        for i, joint_id in enumerate(self._motor_id_list):
            self.data.ctrl[i] = torque[i]

    def close(self):
        self.viewer.close()

    def set_kpkd(self, kp, kd):
        self._motor_kp = kp
        self._motor_kd = kd

    # ==========================================================
    # SENSOR ACCESS
    # ==========================================================

    def get_motor_angles(self):
        return self.data.qpos[7:7+NUM_MOTORS].copy()

    def get_motor_velocities(self):
        return self.data.qvel[6:6+NUM_MOTORS].copy()

    def get_motor_torques(self):
        return self.data.qfrc_actuator[:NUM_MOTORS].copy()

    def get_base_position(self):
        pos = self.data.qpos[0:3]
        return pos.copy()

    def get_base_orientation(self):
        return self.data.qpos[3:7].copy()

    def get_base_orientation_euler(self):
        quat = self.get_base_orientation()
        # quat[0] *= -1
        # quat[2] *= -1
        euler = Rotation.from_quat(quat).as_euler('xyz', degrees=False)
        euler_correct = np.array([euler[2], -euler[1], -euler[0]])

        return euler_correct.copy()

    def get_base_lin_vel(self):
        return self.data.qvel[0:3].copy()
    
    def get_base_lin_acc(self):
        return self.data.sensor('accelerometer').data.copy()

    def get_base_ang_vel(self):
        ang_vel = self.data.qvel[3:6].copy()
        # ang_vel[2] *= -1  # correct yaw direction

        # quat = self.get_base_orientation()
        # R_world_to_base = quat2mat(quat)
        # ang_vel_base = R_world_to_base @ ang_vel_world

        # return ang_vel_base.copy()
        return ang_vel.copy()
    
    def get_contact_flags(self):
        """
        Returns:
            contact_flags: [R1, L1, R2, L2] boolean flags
        """

        contact_flags = [False] * 4

        # предполагаем, что геометрии стоп имеют имена:
        # foot_R1, foot_L1, foot_R2, foot_L2
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_geom_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in foot_body_names
        ]

        for i in range(self.data.ncon):
            contact = self.data.contact[i]

            geom1 = contact.geom1
            geom2 = contact.geom2

            contact_info = {
                "geom1": geom1,
                "geom2": geom2,
                "pos": contact.pos.copy(),
                "frame": contact.frame.copy(),
                "force": None
            }

            # вычисляем контактную силу
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            contact_info["force"] = force.copy()

            # проверяем, есть ли контакт стоп
            for leg_index, foot_id in enumerate(foot_geom_ids):
                if geom1 == foot_id or geom2 == foot_id:
                    contact_flags[leg_index] = True

        return contact_flags
    
    def get_global_foot_positions(self):
        """
        Args:
            leg_index: 0 - R1, 1 - L1, 2 - R2, 3 - L2
        Returns:
            foot_pos: [x, y, z] position of the foot in world frame
        """
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_pos = [0]*4

        for leg_index in range(4):
            foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, foot_body_names[leg_index])
            foot_pos[leg_index] = self.data.body(foot_body_id).xpos.copy()
            
        return foot_pos
    
    def get_local_foot_positions(self):
        foot_body_names = ["ef_R1", "ef_L1", "ef_R2", "ef_L2"]
        foot_pos = [0]*4
        base_pos = np.array(self.get_base_position())
        R_base = quat2mat(self.get_base_orientation())

        for leg_index in range(4):
            foot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, foot_body_names[leg_index])
            foot_pos[leg_index] = R_base @ (self.data.body(foot_body_id).xpos.copy() - base_pos)
            
        return foot_pos

    # ==========================================================
    # EXTERNAL DISTURBANCES
    # ==========================================================

    def set_ext_forces_params(self, magn, duration, interval): 
        self._ext_force_magn = magn
        self._ext_force_duration = int(duration / self._time_step)
        self._ext_force_interval = int(interval / self._time_step)

    def _apply_force(self):
        if 100 < (self._env_step_counter%self._ext_force_interval) <= (100+self._ext_force_duration):
            if (self._env_step_counter%self._ext_force_interval) < 102:
                self._force_dir = -self._force_dir
            force = [0, self._force_dir*self._ext_force_magn, 0]

            print(force)
            body_id = self.model.body("base").id
            point = self.data.xpos[body_id] + np.array([0.0, 0.0, 0.01])  # applying force slightly above the center of mass
            force = np.array(force)
            torque = np.zeros(3)

            mujoco.mj_applyFT(
                self.model,
                self.data,
                force,
                torque,
                point,
                body_id,
                self.data.qfrc_applied
            )
        else:
            force = [0, 0, 0]

            # print(force)
            body_id = self.model.body("base").id
            point = self.data.xpos[body_id] + np.array([0.0, 0.0, 0.01])  # applying force slightly above the center of mass
            force = np.array(force)
            torque = np.zeros(3)

            mujoco.mj_applyFT(
                self.model,
                self.data,
                force,
                torque,
                point,
                body_id,
                self.data.qfrc_applied
            )



