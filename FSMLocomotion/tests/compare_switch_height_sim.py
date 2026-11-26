"""Headless MuJoCo + real FSM/MPC comparison of preparation heights.

Run with .mpc_venv/bin/python and the fsm_locomotion_node executable as argument.
Ground-truth MuJoCo observations replace the estimator; all LCM ports are isolated.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time

import lcm
import mujoco
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'Simulator'))
from additional.mjcf import load_mjmodel
from additional.motor_accurate import MotorModel


def run(binary, height, bindings, adaptation):
    sys.path.insert(0, str(bindings))
    from mors_msgs import robot_state_msg, robot_mode_cmd_msg as Cmd, robot_mode_state_msg as State
    from mors_msgs import robot_mode_result_msg as Result, servo_cmd_msg, robot_cmd_msg
    sys.path.pop(0)
    model = load_mjmodel(str(ROOT / 'Simulator/MJCF/mors.xml'), scene='flat')
    model.opt.timestep = .002
    data = mujoco.MjData(model)
    legs = ('R1', 'L1', 'R2', 'L2')
    joints = [model.joint(f'{joint}_joint_{leg}').id for leg in legs for joint in ('abad', 'hip', 'knee')]
    iq, iv = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    feet = [model.body('ef_' + leg).id for leg in legs]
    knees = {model.body('knee_' + leg).id for leg in legs}
    base = model.body('base').id
    folded = np.array([0., -1.57, 2.9, 0., 1.57, -2.9, 0., 1.57, -2.9, 0., -1.57, 2.9])
    data.qpos[:3], data.qpos[3:7], data.qpos[iq] = [0., 0., .1], [1., 0., 0., 0.], folded
    mujoco.mj_forward(model, data)
    data.qpos[2] += .015 - min(data.xpos[foot, 2] for foot in feet)
    mujoco.mj_forward(model, data)
    motor = MotorModel(12)
    servo = servo_cmd_msg()
    mode = None
    results = []
    metrics = None
    peak_pitch = 0.
    minimum_z = math.inf
    knee_time = 0.
    stage = 'standup'
    next_tick = time.monotonic()
    last_observation = 0.

    with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryFile(mode='w+t') as log:
        config = Path(directory)
        shutil.copytree(ROOT / 'config', config, dirs_exist_ok=True)
        settings = yaml.safe_load((config / 'locomotion_controller.yaml').read_text())
        settings['switch_kin_sch']['preparation_height_m'] = height
        (config / 'locomotion_controller.yaml').write_text(yaml.safe_dump(settings))
        channels = yaml.safe_load((config / 'channels.yaml').read_text())
        robot = yaml.safe_load((config / 'robot.yaml').read_text())['robot_config']
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        url = f'udpm://239.255.76.67:{port}?ttl=0'
        bus = lcm.LCM(url)

        def on_servo(_, packet):
            nonlocal servo
            servo = servo_cmd_msg.decode(packet)

        def on_state(_, packet):
            nonlocal mode
            mode = State.decode(packet)

        bus.subscribe(channels['servo_cmd'], on_servo)
        bus.subscribe(channels['robot_mode_state'], on_state)
        bus.subscribe(channels['robot_mode_result'], lambda _, packet: results.append(Result.decode(packet)))
        env = {**os.environ, 'CONFIGPATH': str(config), 'LCM_LOCOMOTION_URL': url, 'LCM_SERVO_URL': url}
        with subprocess.Popen([str(binary)], env=env, stdout=log, stderr=log) as process:
            try:
                def tick():
                    nonlocal next_tick, last_observation, peak_pitch, minimum_z, knee_time
                    delay = next_tick - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    next_tick = max(next_tick + .002, time.monotonic())
                    for _ in range(128):
                        if not bus.handle_timeout(0):
                            break
                    q, v = data.qpos[iq], data.qvel[iv]
                    motor.set_sensor_data(q, v)
                    if mode is None or mode.current_state == State.PASSIVE:
                        motor.set_kp(np.full(12, 12.))
                        motor.set_kd(np.full(12, .4))
                        motor.set_ref_angle(folded)
                        motor.set_ref_vel(np.zeros(12))
                        motor.set_ref_torque(np.zeros(12))
                    else:
                        motor.set_kp(servo.kp)
                        motor.set_kd(servo.kd)
                        motor.set_ref_angle(servo.position)
                        motor.set_ref_vel(servo.velocity)
                        motor.set_ref_torque(np.array(servo.torque) * robot['kt'] / robot['gear_ratio'])
                    data.ctrl[:] = motor.step()
                    mujoco.mj_step(model, data)
                    ground_bodies = set()
                    for contact in data.contact[:data.ncon]:
                        bodies = [int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])]
                        if 0 in bodies:
                            ground_bodies.update(bodies)
                    rotation = data.xmat[base].reshape(3, 3)
                    pitch = math.asin(float(np.clip(-rotation[2, 0], -1., 1.)))
                    if metrics is not None:
                        peak_pitch = max(peak_pitch, abs(pitch))
                        minimum_z = min(minimum_z, float(data.xpos[base, 2]))
                        if knees & ground_bodies:
                            knee_time += model.opt.timestep
                    now = time.monotonic()
                    if now - last_observation >= .004:
                        state = robot_state_msg()
                        state.body.position = data.xpos[base].tolist()
                        quat = data.xquat[base]
                        state.body.orientation_quaternion = [*quat[1:], quat[0]]
                        state.body.orientation_euler = [math.atan2(rotation[2, 1], rotation[2, 2]), pitch,
                                                        math.atan2(rotation[1, 0], rotation[0, 0])]
                        state.body.lin_vel = data.qvel[:3].tolist()
                        state.body.ang_vel = data.qvel[3:6].tolist()
                        state.joints.position = data.qpos[iq].tolist()
                        state.joints.velocity = data.qvel[iv].tolist()
                        state.joints.torque = data.qfrc_actuator[iv].tolist()
                        for leg, foot in zip(legs, feet):
                            setattr(state.legs, leg.lower() + '_pos', data.xpos[foot].tolist())
                            velocity = np.zeros(6)
                            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, foot, velocity, 0)
                            setattr(state.legs, leg.lower() + '_vel', velocity[3:].tolist())
                        state.legs.contact_states = [foot in ground_bodies for foot in feet]
                        state.readiness = [True] * 19
                        bus.publish(channels['robot_state'], state.encode())
                        last_observation = now
                    if process.poll() is not None:
                        raise RuntimeError('FSM process exited')

                def wait(predicate, timeout):
                    end = time.monotonic() + timeout
                    while not predicate():
                        tick()
                        if mode and mode.fault:
                            raise RuntimeError(mode.fault_str)
                        if time.monotonic() >= end:
                            raise RuntimeError('Simulation timed out')

                def command(code, target=0, params=0, timeout=30.):
                    results.clear()
                    message = Cmd()
                    message.command, message.target, message.params = code, target, params
                    bus.publish(channels['robot_mode_cmd'], message.encode())
                    wait(lambda: any(r.result != Result.ACCEPTED for r in results), timeout)
                    final = next(r for r in results if r.result != Result.ACCEPTED)
                    if final.result != Result.SUCCEEDED:
                        raise RuntimeError(final.reason)

                wait(lambda: mode is not None, 3.)
                pose = robot_cmd_msg()
                pose.cmd_pose[2], pose.adaptation_type = .20, adaptation
                pose.active_legs = [True] * 4
                bus.publish(channels['robot_cmd'], pose.encode())
                command(Cmd.SET_MODE, Cmd.STANDING, timeout=30.)
                stage = '>< -> <<'
                command(Cmd.START_ACTION, Cmd.CHANGE_KINEMATICS, Cmd.KNEE_BACK)
                metrics = []
                for goal, label in ((Cmd.KNEE_OUTSIDE, '<< -> <>'), (Cmd.KNEE_BACK, '<> -> <<')):
                    stage = label
                    peak_pitch, minimum_z, knee_time = 0., math.inf, 0.
                    command(Cmd.START_ACTION, Cmd.CHANGE_KINEMATICS, goal)
                    metrics.append(dict(transition=label, max_abs_pitch_rad=round(peak_pitch, 4),
                                        min_body_z_m=round(minimum_z, 4), knee_ground_contact_s=round(knee_time, 3)))
                return dict(preparation_height_m=height, adaptation_type=adaptation, transitions=metrics)
            except RuntimeError as error:
                return dict(preparation_height_m=height, adaptation_type=adaptation, error=str(error),
                            stage=stage, max_abs_pitch_rad=round(peak_pitch, 4), knee_ground_contact_s=round(knee_time, 3),
                            body_z_m=float(data.xpos[base, 2]), joints=data.qpos[iq].tolist())
            finally:
                process.terminate()
                process.wait(timeout=3.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('binary', type=Path)
    parser.add_argument('--heights', nargs='+', type=float, default=[.20, .27])
    parser.add_argument('--adaptation', type=int, choices=[0, 1, 2], default=2)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as directory:
        subprocess.run(['lcm-gen', '-p', '--ppath', directory,
                        *map(str, sorted((ROOT / 'lcm_msgs').glob('*.lcm')))], check=True)
        report = [run(args.binary.resolve(), height, Path(directory), args.adaptation) for height in args.heights]
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
