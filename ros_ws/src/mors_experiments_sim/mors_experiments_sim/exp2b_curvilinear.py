import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# Experiment 2b: open-loop walk along the X and Y axes without turning - a series
# of SEGMENT_TIME segments, each with its own (vx, vy). Velocity changes linearly
# over RAMP_TIME at segment boundaries.

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
REF_VELOCITY_X = 0.3
REF_VELOCITY_Y = 0.3

# Segment velocities (vx, vy) in units of (ref_velocity_x, ref_velocity_y).
SEGMENT_DIRECTIONS = [
    (0.0, 1.0),
    (1.0, -1.0),
    (0.0, -1.0),
    (-1.0, 1.0),
]

# Scenario phase durations (s). Stand-up and lay-down actions run inside
# robot_mode_controller (~2.5 s and ~3 s) without completion feedback, so their
# phases are sized with a margin.
LIE_TIME = 1.0
STANDUP_TIME = 3.0
STAND_TIME = 1.0
STEP_IN_PLACE_TIME = 1.0
SEGMENT_TIME = 5.0
RAMP_TIME = 1.0         # linear transition between segment velocities and to 0 on stop
LAY_DOWN_TIME = 3.5

TIMER_PERIOD = 0.02

PHASES = [
    ("LIE", LIE_TIME),
    ("STANDUP", STANDUP_TIME),
    ("STAND_BEFORE", STAND_TIME),
    ("STEP_BEFORE", STEP_IN_PLACE_TIME),
    *[("WALK", SEGMENT_TIME)] * len(SEGMENT_DIRECTIONS),
    ("STOP", RAMP_TIME),
    ("STEP_AFTER", STEP_IN_PLACE_TIME),
    ("STAND_AFTER", STAND_TIME),
    ("LAY_DOWN", LAY_DOWN_TIME),
    ("LIE_END", LIE_TIME),
]


class AxisWalkExperiment(Node):
    def __init__(self):
        super().__init__("exp2b_curvilinear")

        # Defaults above can be overridden at startup: --ros-args -p ref_velocity_y:=0.2
        self.body_z = self.declare_parameter("body_z", ROBOT_BODY_Z).value
        self.gait_offsets = list(self.declare_parameter("gait_offsets", GAIT_TYPE).value)
        self.t_sw = self.declare_parameter("t_sw", T_SW).value
        self.t_st = self.declare_parameter("t_st", T_ST).value
        self.stride_height = self.declare_parameter("stride_height", STRIDE_HEIGHT).value
        self.ref_velocity_x = self.declare_parameter("ref_velocity_x", REF_VELOCITY_X).value
        self.ref_velocity_y = self.declare_parameter("ref_velocity_y", REF_VELOCITY_Y).value

        self.segment_velocities = [
            (kx * self.ref_velocity_x, ky * self.ref_velocity_y) for kx, ky in SEGMENT_DIRECTIONS
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
        self.segment_idx = -1

        self.get_logger().info(
            f"Experiment 2b: axis walk | body_z={self.body_z} gait={self.gait_offsets} "
            f"t_sw={self.t_sw} t_st={self.t_st} stride_h={self.stride_height} "
            f"segments={self.segment_velocities} segment_time={SEGMENT_TIME}"
        )
        self._enter_phase()

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def send_request(self, cli, data: int):
        req = RobotCmd.Request()
        req.data = data
        cli.call_async(req)

    def _enter_phase(self):
        name = PHASES[self.phase_idx][0]

        if name == "WALK":
            self.segment_idx += 1
            vx, vy = self.segment_velocities[self.segment_idx]
            self.get_logger().info(
                f"Phase: WALK {self.segment_idx + 1}/{len(self.segment_velocities)} "
                f"vx={vx} vy={vy}")
        else:
            self.get_logger().info(f"Phase: {name}")

        if name == "STANDUP":
            self.send_request(self.action_cli, STANDUP)
            self.send_request(self.mode_cli, LOCOMOTION_MODE)
        elif name == "LAY_DOWN":
            self.send_request(self.action_cli, LAY_DOWN)
            self.send_request(self.mode_cli, DO_NOTHING_MODE)

    def _ref_velocity(self, name: str) -> tuple[float, float]:
        if name == "WALK":
            v_from = self.segment_velocities[self.segment_idx - 1] if self.segment_idx > 0 else (0.0, 0.0)
            v_to = self.segment_velocities[self.segment_idx]
        elif name == "STOP":
            v_from, v_to = self.segment_velocities[-1], (0.0, 0.0)
        else:
            return 0.0, 0.0
        k = min(self.phase_time / RAMP_TIME, 1.0)
        return tuple(a + k * (b - a) for a, b in zip(v_from, v_to))

    def timer_callback(self):
        name, duration = PHASES[self.phase_idx]

        self.cmd_vel_msg.linear.x, self.cmd_vel_msg.linear.y = self._ref_velocity(name)
        self.gait_params_msg.standing = name not in ("STEP_BEFORE", "WALK", "STOP", "STEP_AFTER")

        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.cmd_pose_pub.publish(self.cmd_pose_msg)
        self.gait_params_pub.publish(self.gait_params_msg)

        self.phase_time += TIMER_PERIOD
        if self.phase_time < duration - 0.5 * TIMER_PERIOD:
            return

        self.phase_time = 0.0
        self.phase_idx += 1
        if self.phase_idx == len(PHASES):
            self.get_logger().info("Experiment finished")
            self.timer.cancel()
            self.finished = True
            return

        self._enter_phase()


def main(args=None):
    rclpy.init(args=args)

    experiment = AxisWalkExperiment()

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
