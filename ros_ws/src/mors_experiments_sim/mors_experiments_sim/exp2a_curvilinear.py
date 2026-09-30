import math

import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# Experiment 2a: curvilinear walk closed-loop on odom - go SIDE_LENGTH along the
# local X axis, turn in place by each of TURN_ANGLES in turn, going SIDE_LENGTH
# after every turn. Distance is the displacement projected onto the body X axis
# at the segment start. Turn targets are absolute: yaw0 plus the sum of the turns
# made so far, where yaw0 is the heading before the first segment. Heading is not
# corrected while walking.

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

SIDE_LENGTH = 1.0
TURN_ANGLES = [math.pi / 2, -math.pi / 2]   # rad, > 0 turns left (CCW)

# Scenario phase durations (s). Stand-up and lay-down actions run inside
# robot_mode_controller (~2.5 s and ~3 s) without completion feedback, so their
# phases are sized with a margin.
LIE_TIME = 1.0
STANDUP_TIME = 3.0
STAND_TIME = 1.0
STEP_IN_PLACE_TIME = 1.0
LAY_DOWN_TIME = 3.5

# WALK and TURN end when the odom target is reached. Velocity ramps up linearly
# over RAMP_TIME and brakes with the same deceleration (v = sqrt(2 * a * remaining)),
# but not below MIN_VELOCITY_* so the target is reached in finite time.
RAMP_TIME = 1.0
MIN_VELOCITY_X = 0.03       # m/s
MIN_VELOCITY_Z = 0.05       # rad/s
DISTANCE_TOLERANCE = 0.005  # m
YAW_TOLERANCE = math.radians(0.5)
MOTION_TIMEOUT = 30.0       # s, safety limit for a single WALK/TURN

TIMER_PERIOD = 0.02

PHASES = [
    ("LIE", LIE_TIME),
    ("STANDUP", STANDUP_TIME),
    ("STAND_BEFORE", STAND_TIME),
    ("STEP_BEFORE", STEP_IN_PLACE_TIME),
    ("WALK", MOTION_TIMEOUT),
    *[p for _ in TURN_ANGLES for p in (("TURN", MOTION_TIMEOUT), ("WALK", MOTION_TIMEOUT))],
    ("STEP_AFTER", STEP_IN_PLACE_TIME),
    ("STAND_AFTER", STAND_TIME),
    ("LAY_DOWN", LAY_DOWN_TIME),
    ("LIE_END", LIE_TIME),
]


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def motion_profile(t: float, remaining: float, ref: float, min_vel: float) -> float:
    """Speed magnitude for a move with `remaining` distance left at time `t` since start."""
    accel = ref / RAMP_TIME
    return max(min(ref, accel * t, math.sqrt(2.0 * accel * remaining)), min_vel)


class CurvilinearExperiment(Node):
    def __init__(self):
        super().__init__("exp2a_curvilinear")

        # Defaults above can be overridden at startup: --ros-args -p ref_velocity_x:=0.2
        self.body_z = self.declare_parameter("body_z", ROBOT_BODY_Z).value
        self.gait_offsets = list(self.declare_parameter("gait_offsets", GAIT_TYPE).value)
        self.t_sw = self.declare_parameter("t_sw", T_SW).value
        self.t_st = self.declare_parameter("t_st", T_ST).value
        self.stride_height = self.declare_parameter("stride_height", STRIDE_HEIGHT).value
        self.ref_velocity_x = self.declare_parameter("ref_velocity_x", REF_VELOCITY_X).value
        self.ref_velocity_z = self.declare_parameter("ref_velocity_z", REF_VELOCITY_Z).value

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

        self.odom = None
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        for cli in (self.mode_cli, self.action_cli):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for service '{cli.srv_name}'...")
        while self.odom is None:
            self.get_logger().info("Waiting for topic 'odom'...")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.phase_idx = 0
        self.phase_time = 0.0
        self.finished = False

        self.yaw0 = None
        self.turn_count = 0
        self.segment_start = (0.0, 0.0, 0.0)   # x, y, yaw at WALK start
        self.target_yaw = 0.0

        self.get_logger().info(
            f"Experiment 2a: curvilinear | body_z={self.body_z} gait={self.gait_offsets} "
            f"t_sw={self.t_sw} t_st={self.t_st} stride_h={self.stride_height} vx={self.ref_velocity_x} "
            f"wz={self.ref_velocity_z} side={SIDE_LENGTH} "
            f"turns={[round(math.degrees(a), 1) for a in TURN_ANGLES]}"
        )
        self._enter_phase()

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)

    def send_request(self, cli, data: int):
        req = RobotCmd.Request()
        req.data = data
        cli.call_async(req)

    def _enter_phase(self):
        name = PHASES[self.phase_idx][0]
        self.get_logger().info(f"Phase {self.phase_idx + 1}/{len(PHASES)}: {name}")

        if name == "STANDUP":
            self.send_request(self.action_cli, STANDUP)
            self.send_request(self.mode_cli, LOCOMOTION_MODE)
        elif name == "LAY_DOWN":
            self.send_request(self.action_cli, LAY_DOWN)
            self.send_request(self.mode_cli, DO_NOTHING_MODE)
        elif name == "WALK":
            self.segment_start = self.odom
            if self.yaw0 is None:
                self.yaw0 = self.odom[2]
        elif name == "TURN":
            self.turn_count += 1
            self.target_yaw = wrap_angle(self.yaw0 + sum(TURN_ANGLES[:self.turn_count]))

    def _walk_progress(self):
        """Returns (along, lateral) displacement in the segment start frame."""
        x0, y0, yaw_s = self.segment_start
        dx, dy = self.odom[0] - x0, self.odom[1] - y0
        c, s = math.cos(yaw_s), math.sin(yaw_s)
        return c * dx + s * dy, -s * dx + c * dy

    def _walk_command(self) -> bool:
        """Sets cmd_vel for WALK, returns True when the segment is done."""
        remaining = SIDE_LENGTH - self._walk_progress()[0]
        if remaining <= DISTANCE_TOLERANCE:
            return True
        self.cmd_vel_msg.linear.x = motion_profile(
            self.phase_time, remaining, self.ref_velocity_x, MIN_VELOCITY_X)
        return False

    def _turn_command(self) -> bool:
        """Sets cmd_vel for TURN, returns True when the target heading is reached.

        Like WALK, an overshoot ends the turn instead of turning back.
        """
        direction = math.copysign(1.0, TURN_ANGLES[self.turn_count - 1])
        remaining = direction * wrap_angle(self.target_yaw - self.odom[2])
        if remaining <= YAW_TOLERANCE:
            return True
        self.cmd_vel_msg.angular.z = direction * motion_profile(
            self.phase_time, remaining, self.ref_velocity_z, MIN_VELOCITY_Z)
        return False

    def _log_motion_result(self, name: str):
        if name == "WALK":
            along, lateral = self._walk_progress()
            yaw_drift = wrap_angle(self.odom[2] - self.segment_start[2])
            self.get_logger().info(
                f"WALK done in {self.phase_time:.2f} s: along={along:.3f} m "
                f"lateral={lateral:.3f} m yaw_drift={math.degrees(yaw_drift):.2f} deg")
        elif name == "TURN":
            yaw_err = wrap_angle(self.odom[2] - self.target_yaw)
            self.get_logger().info(
                f"TURN {self.turn_count} done in {self.phase_time:.2f} s: "
                f"yaw_err={math.degrees(yaw_err):.2f} deg")

    def timer_callback(self):
        name, duration = PHASES[self.phase_idx]

        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        target_reached = False
        if name == "WALK":
            target_reached = self._walk_command()
        elif name == "TURN":
            target_reached = self._turn_command()
        self.gait_params_msg.standing = name not in ("STEP_BEFORE", "WALK", "TURN", "STEP_AFTER")

        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.cmd_pose_pub.publish(self.cmd_pose_msg)
        self.gait_params_pub.publish(self.gait_params_msg)

        if target_reached:
            self._log_motion_result(name)
        else:
            self.phase_time += TIMER_PERIOD
            if self.phase_time < duration - 0.5 * TIMER_PERIOD:
                return
            if name in ("WALK", "TURN"):
                self.get_logger().warn(f"{name} timed out after {MOTION_TIMEOUT} s")
                self._log_motion_result(name)

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

    experiment = CurvilinearExperiment()

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
