import math

import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# Experiment 2c: open-loop walk along a circle of radius vx/wz for N_LAPS laps.
# vx and wz are ramped together so the radius stays constant.

DO_NOTHING_MODE = 0
LOCOMOTION_MODE = 1

STANDUP = 1
LAY_DOWN = 2

# ROS parameter defaults
ROBOT_BODY_Z = 0.2
GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]
T_SW = 0.18
T_ST = 0.35
STRIDE_HEIGHT = 0.06
REF_VELOCITY_X = 0.0    # m/s
REF_VELOCITY_Z = 0.7    # rad/s, > 0 turns left (CCW)

N_LAPS = 1

# Scenario phase durations (s). Stand-up and lay-down actions run inside
# robot_mode_controller (~2.5 s and ~3 s) without completion feedback, so their
# phases are sized with a margin.
LIE_TIME = 1.0
STANDUP_TIME = 3.0
STAND_TIME = 1.0
STEP_IN_PLACE_TIME = 1.0
RAMP_TIME = 1.0         # linear 0 -> ref velocities at walk start and back on stop
LAY_DOWN_TIME = 3.5

TIMER_PERIOD = 0.02


class CircleWalkExperiment(Node):
    def __init__(self):
        super().__init__("exp2c_curvilinear")

        # Defaults above can be overridden at startup: --ros-args -p ref_velocity_z:=0.5
        self.body_z = self.declare_parameter("body_z", ROBOT_BODY_Z).value
        self.gait_offsets = list(self.declare_parameter("gait_offsets", GAIT_TYPE).value)
        self.t_sw = self.declare_parameter("t_sw", T_SW).value
        self.t_st = self.declare_parameter("t_st", T_ST).value
        self.stride_height = self.declare_parameter("stride_height", STRIDE_HEIGHT).value
        self.ref_velocity_x = self.declare_parameter("ref_velocity_x", REF_VELOCITY_X).value
        self.ref_velocity_z = self.declare_parameter("ref_velocity_z", REF_VELOCITY_Z).value

        # The ramp-up at WALK start loses wz * RAMP_TIME / 2 of heading and the STOP ramp
        # adds the same back, so the total turn is wz * walk_time = N_LAPS * 2 * pi.
        self.walk_time = N_LAPS * 2.0 * math.pi / abs(self.ref_velocity_z)

        self.phases = [
            ("LIE", LIE_TIME),
            ("STANDUP", STANDUP_TIME),
            ("STAND_BEFORE", STAND_TIME),
            ("STEP_BEFORE", STEP_IN_PLACE_TIME),
            ("WALK", self.walk_time),
            ("STOP", RAMP_TIME),
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

        self.get_logger().info(
            f"Experiment 2c: circle walk | body_z={self.body_z} gait={self.gait_offsets} "
            f"t_sw={self.t_sw} t_st={self.t_st} stride_h={self.stride_height} vx={self.ref_velocity_x} "
            f"wz={self.ref_velocity_z} radius={self.ref_velocity_x / abs(self.ref_velocity_z):.3f} "
            f"laps={N_LAPS} walk_time={self.walk_time:.2f}"
        )
        self._enter_phase()

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def send_request(self, cli, data: int):
        req = RobotCmd.Request()
        req.data = data
        cli.call_async(req)

    def _enter_phase(self):
        name = self.phases[self.phase_idx][0]
        self.get_logger().info(f"Phase: {name}")

        if name == "STANDUP":
            self.send_request(self.action_cli, STANDUP)
            self.send_request(self.mode_cli, LOCOMOTION_MODE)
        elif name == "LAY_DOWN":
            self.send_request(self.action_cli, LAY_DOWN)
            self.send_request(self.mode_cli, DO_NOTHING_MODE)

    def _ramp(self, name: str) -> float:
        """Common scale in [0, 1] for the reference velocities."""
        if name == "WALK":
            return min(self.phase_time / RAMP_TIME, 1.0)
        if name == "STOP":
            return max(1.0 - self.phase_time / RAMP_TIME, 0.0)
        return 0.0

    def timer_callback(self):
        name, duration = self.phases[self.phase_idx]

        k = self._ramp(name)
        self.cmd_vel_msg.linear.x = k * self.ref_velocity_x
        self.cmd_vel_msg.angular.z = k * self.ref_velocity_z
        self.gait_params_msg.standing = name not in ("STEP_BEFORE", "WALK", "STOP", "STEP_AFTER")

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

    experiment = CircleWalkExperiment()

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
