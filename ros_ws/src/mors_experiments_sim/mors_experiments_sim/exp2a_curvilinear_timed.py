import math

import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# Experiment 2a, open-loop variant of exp2a_curvilinear for hardware, where odom
# is not available while the robot lies: walk along the local X axis for walk_time,
# turn in place by each of TURN_ANGLES in turn, walking walk_time after every turn.
# Every WALK and TURN is a trapezoidal velocity profile: linear ramp up over
# RAMP_TIME, cruise, linear ramp down to 0 over RAMP_TIME at the phase end.
# A TURN lasts |angle| / wz + RAMP_TIME, so the nominal heading change is the angle.

DO_NOTHING_MODE = 0
LOCOMOTION_MODE = 1

STANDUP = 1
LAY_DOWN = 2

# ROS parameter defaults
ROBOT_BODY_Z = 0.2
GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]
T_SW = 0.26
T_ST = 0.35
STRIDE_HEIGHT = 0.06
REF_VELOCITY_X = 0.3    # m/s
REF_VELOCITY_Z = 0.5    # rad/s, yaw rate magnitude; turn direction is the sign of the turn angle
WALK_TIME = 5.0         # s, duration of every WALK segment, ramps included

TURN_ANGLES = [math.pi / 2, -math.pi / 2]   # rad, > 0 turns left (CCW)

# Scenario phase durations (s). Stand-up and lay-down actions run inside
# robot_mode_controller (~2.5 s and ~3 s) without completion feedback, so their
# phases are sized with a margin.
LIE_TIME = 1.0
STANDUP_TIME = 3.0
STAND_TIME = 1.0
STEP_IN_PLACE_TIME = 1.0
RAMP_TIME = 1.0
LAY_DOWN_TIME = 3.5

TIMER_PERIOD = 0.02


class CurvilinearTimedExperiment(Node):
    def __init__(self):
        super().__init__("exp2a_curvilinear_timed")

        # Defaults above can be overridden at startup: --ros-args -p walk_time:=4.0
        self.body_z = self.declare_parameter("body_z", ROBOT_BODY_Z).value
        self.gait_offsets = list(self.declare_parameter("gait_offsets", GAIT_TYPE).value)
        self.t_sw = self.declare_parameter("t_sw", T_SW).value
        self.t_st = self.declare_parameter("t_st", T_ST).value
        self.stride_height = self.declare_parameter("stride_height", STRIDE_HEIGHT).value
        self.ref_velocity_x = self.declare_parameter("ref_velocity_x", REF_VELOCITY_X).value
        self.ref_velocity_z = self.declare_parameter("ref_velocity_z", REF_VELOCITY_Z).value
        self.walk_time = self.declare_parameter("walk_time", WALK_TIME).value

        if self.walk_time < 2.0 * RAMP_TIME:
            raise ValueError(f"walk_time={self.walk_time} must be >= 2 * RAMP_TIME ({2.0 * RAMP_TIME})")

        turn_times = [abs(a) / abs(self.ref_velocity_z) + RAMP_TIME for a in TURN_ANGLES]
        self.phases = [
            ("LIE", LIE_TIME),
            ("STANDUP", STANDUP_TIME),
            ("STAND_BEFORE", STAND_TIME),
            ("STEP_BEFORE", STEP_IN_PLACE_TIME),
            ("WALK", self.walk_time),
            *[p for t in turn_times for p in (("TURN", t), ("WALK", self.walk_time))],
            ("STEP_AFTER", STEP_IN_PLACE_TIME),
            ("STAND_AFTER", STAND_TIME),
            ("LAY_DOWN", LAY_DOWN_TIME),
            ("LIE_END", LIE_TIME),
        ]

        self.mode_cli = self.create_client(RobotCmd, "robot_mode")
        self.action_cli = self.create_client(RobotCmd, "robot_action")

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()
        self.cmd_pose_pub = self.create_publisher(Twist, "cmd_pose", 10)
        self.cmd_pose_msg = Twist()
        self.cmd_pose_msg.linear.z = self.body_z
        self.gait_params_pub = self.create_publisher(GaitParams, "gait_params", 10)
        self.gait_params_msg = GaitParams()
        self.gait_params_msg.standing = True
        self.gait_params_msg.stride_height = self.stride_height
        self.gait_params_msg.t_st = self.t_st
        self.gait_params_msg.t_sw = self.t_sw
        self.gait_params_msg.gait_offsets = self.gait_offsets

        for cli in (self.mode_cli, self.action_cli):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for service '{cli.srv_name}'...")

        self.phase_idx = 0
        self.phase_time = 0.0
        self.finished = False
        self.turn_count = 0

        self.get_logger().info(
            f"Experiment 2a: curvilinear timed | body_z={self.body_z} gait={self.gait_offsets} "
            f"t_sw={self.t_sw} t_st={self.t_st} stride_h={self.stride_height} vx={self.ref_velocity_x} "
            f"wz={self.ref_velocity_z} walk_time={self.walk_time} "
            f"turns={[round(math.degrees(a), 1) for a in TURN_ANGLES]} "
            f"turn_times={[round(t, 2) for t in turn_times]}"
        )
        self._enter_phase()

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def send_request(self, cli, data: int):
        req = RobotCmd.Request()
        req.data = data
        cli.call_async(req)

    def _enter_phase(self):
        name = self.phases[self.phase_idx][0]
        self.get_logger().info(f"Phase {self.phase_idx + 1}/{len(self.phases)}: {name}")

        if name == "STANDUP":
            self.send_request(self.action_cli, STANDUP)
            self.send_request(self.mode_cli, LOCOMOTION_MODE)
        elif name == "LAY_DOWN":
            self.send_request(self.action_cli, LAY_DOWN)
            self.send_request(self.mode_cli, DO_NOTHING_MODE)
        elif name == "TURN":
            self.turn_count += 1

    def _ramp(self, duration: float) -> float:
        """Trapezoidal profile scale in [0, 1] at phase_time of a phase lasting `duration`."""
        return max(min(1.0, self.phase_time / RAMP_TIME, (duration - self.phase_time) / RAMP_TIME), 0.0)

    def timer_callback(self):
        name, duration = self.phases[self.phase_idx]

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        if name == "WALK":
            self.cmd_vel_msg.linear.x = self.ref_velocity_x * self._ramp(duration)
        elif name == "TURN":
            direction = math.copysign(1.0, TURN_ANGLES[self.turn_count - 1])
            self.cmd_vel_msg.angular.z = direction * abs(self.ref_velocity_z) * self._ramp(duration)
        self.gait_params_msg.standing = name not in ("STEP_BEFORE", "WALK", "TURN", "STEP_AFTER")

        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.cmd_pose_pub.publish(self.cmd_pose_msg)
        self.gait_params_pub.publish(self.gait_params_msg)

        self.phase_time += TIMER_PERIOD
        if self.phase_time < duration - 0.5 * TIMER_PERIOD:
            return

        self.phase_time = 0.0
        self.phase_idx += 1
        if self.phase_idx == len(self.phases):
            self.get_logger().info("Experiment finished")
            self.timer.cancel()
            self.finished = True
            return

        self._enter_phase()


def main(args=None):
    rclpy.init(args=args)

    experiment = CurvilinearTimedExperiment()

    try:
        while rclpy.ok() and not experiment.finished:
            rclpy.spin_once(experiment)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        experiment.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
