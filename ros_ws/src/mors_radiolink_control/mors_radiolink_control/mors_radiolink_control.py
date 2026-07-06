from numbers import Real

import rclpy
from geometry_msgs.msg import Twist
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
from rclpy.node import Node
from sensor_msgs.msg import Joy


DO_NOTHING_MODE = 0
LOCOMOTION_MODE = 1
BODY_MODE = 2
STANDING_MODE = 3

NO_ACTION = 0
STANDUP = 1
LAY_DOWN = 2

BUTTON_RELEASED = 0
BUTTON_PRESSED = 1

# Radiolink T8S axis layout (axis index == channel - 1), verified empirically.
AXIS_X = 2            # left stick vertical  -> forward/backward
AXIS_Y = 3            # left stick horizontal -> lateral
AXIS_YAW = 0          # right stick horizontal -> rotation about Z
AXIS_PITCH = 1        # right stick vertical -> body pitch (BODY_MODE only)
AXIS_MODE = 4         # CH5 3-position switch -> mode select
AXIS_STANDUP = 5      # CH6 button -> stand up / lay down
AXIS_SPEED = 6        # CH7 3-position switch -> max X speed
AXIS_STRIDE = 7       # CH8 knob -> stride height

# 3-position switch decoding. On this transmitter the physical top position reads
# axis < -0.5 and the bottom reads axis > 0.5 (middle in between); swap the branches
# again if a channel gets reversed.
SWITCH_THRESHOLD = 0.5

ROBOT_BODY_Z = 0.2
GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]

# CH7 max forward speed by switch position (m/s).
MAX_SPEED_TOP = 0.3
MAX_SPEED_MIDDLE = 0.5
MAX_SPEED_BOTTOM = 0.9

# Lateral and angular limits are derived from the forward speed the same way as
# mors_keyboard_control does.
LATERAL_SPEED_RATIO = 0.0
ANGULAR_SPEED_RATIO = 2.0

# Gait timing: t_sw fixed, t_st interpolated from speed (same as mors_keyboard_control).
T_SW = 0.2
T_ST_MAX = 0.4
T_ST_MIN = 0.2
SPEED_MIN = 0.1
SPEED_MAX = MAX_SPEED_BOTTOM

# CH8 knob stride-height range (m).
STRIDE_HEIGHT_MIN = 0.035
STRIDE_HEIGHT_MAX = 0.08

BODY_ROLL_MAX = 0.907
BODY_PITCH_MAX = 0.45
BODY_YAW_MAX = 0.6

JOY_DEADZONE = 0.04
FLOAT_EPS = 1e-6
CMD_POSE_LPF_ALPHA = 0.2
# Resend cmd_pose for a short window after leaving DO_NOTHING so the controller
# receives the setpoint after it has actually switched mode (~0.5 s at 50 Hz).
CMD_POSE_STAND_REPUBLISH_TICKS = 25
# While the robot is lying down, resend cmd_pose every 2 s (100 ticks at 50 Hz)
# so the controller keeps receiving the lay-down pose setpoint.
CMD_POSE_SLEEP_REPUBLISH_TICKS = 100


class MorsRadiolinkControl(Node):
    def __init__(self):
        super().__init__("mors_radiolink_control")

        self.mode_cli = self.create_client(RobotCmd, "robot_mode")
        self.req = RobotCmd.Request()

        self.action_cli = self.create_client(RobotCmd, "robot_action")

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()
        self.cmd_pose_pub = self.create_publisher(Twist, "cmd_pose", 10)
        self.cmd_pose_msg = Twist()
        self.gait_params_pub = self.create_publisher(GaitParams, "gait_params", 10)
        self.gait_params_msg = GaitParams()
        self.max_vel_x = MAX_SPEED_TOP
        self.stride_height = STRIDE_HEIGHT_MIN
        self.cmd_pose_msg.linear.z = ROBOT_BODY_Z
        self.gait_params_msg.standing = True
        self.gait_params_msg.stride_height = self.stride_height
        self.gait_params_msg.t_st = self._t_st_for_speed(self.max_vel_x)
        self.gait_params_msg.t_sw = T_SW
        self.gait_params_msg.gait_offsets = GAIT_TYPE

        self.prev_cmd_vel_state = None
        self.prev_cmd_pose_state = None
        self.prev_gait_params_state = None
        self.filtered_cmd_pose_state = None
        self.cmd_pose_force_ticks = 0
        self.cmd_pose_sleep_ticks = 0

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.is_sleeping = True
        self.robot_mode = DO_NOTHING_MODE
        self.prev_robot_mode = DO_NOTHING_MODE
        self.action_num = NO_ACTION

        self.mode_switch = STANDING_MODE
        self.standup_button = BUTTON_RELEASED
        self.prev_standup_button = BUTTON_RELEASED

        self.stick_x = 0.0
        self.stick_y = 0.0
        self.stick_yaw = 0.0
        self.stick_pitch = 0.0
        self.stick_roll = 0.0

        print("[RADIOLINK_TELEOP]: Started")

    def _t_st_for_speed(self, speed: float) -> float:
        clamped_speed = min(max(speed, SPEED_MIN), SPEED_MAX)
        speed_span = SPEED_MAX - SPEED_MIN
        if speed_span <= FLOAT_EPS:
            return T_ST_MAX

        speed_ratio = (clamped_speed - SPEED_MIN) / speed_span
        return T_ST_MAX + speed_ratio * (T_ST_MIN - T_ST_MAX)

    def joy_callback(self, msg: Joy):
        if msg.axes[AXIS_STANDUP] > SWITCH_THRESHOLD:
            self.standup_button = BUTTON_RELEASED
        elif msg.axes[AXIS_STANDUP] < -SWITCH_THRESHOLD:
            self.standup_button = BUTTON_PRESSED

        if msg.axes[AXIS_MODE] > SWITCH_THRESHOLD:
            self.mode_switch = LOCOMOTION_MODE
        elif abs(msg.axes[AXIS_MODE]) < SWITCH_THRESHOLD:
            self.mode_switch = STANDING_MODE
        elif msg.axes[AXIS_MODE] < -SWITCH_THRESHOLD:
            self.mode_switch = BODY_MODE

        if msg.axes[AXIS_SPEED] > SWITCH_THRESHOLD:
            self.max_vel_x = MAX_SPEED_BOTTOM
        elif abs(msg.axes[AXIS_SPEED]) < SWITCH_THRESHOLD:
            self.max_vel_x = MAX_SPEED_MIDDLE
        elif msg.axes[AXIS_SPEED] < -SWITCH_THRESHOLD:
            self.max_vel_x = MAX_SPEED_TOP

        self.stride_height = STRIDE_HEIGHT_MIN + (msg.axes[AXIS_STRIDE] + 1.0) * (
            STRIDE_HEIGHT_MAX - STRIDE_HEIGHT_MIN
        ) / 2.0

        self.stick_x = msg.axes[AXIS_X]
        self.stick_y = msg.axes[AXIS_Y]
        self.stick_yaw = msg.axes[AXIS_YAW]
        self.stick_pitch = -msg.axes[AXIS_PITCH]
        self.stick_roll = msg.axes[AXIS_Y]

    def send_mode_request(self, mode: int):
        self.req.data = mode
        resp = self.mode_cli.call_async(self.req)
        return resp.result()

    def send_action_request(self, mode: int):
        self.req.data = mode
        resp = self.action_cli.call_async(self.req)
        return resp.result()

    def _values_changed(self, prev, current):
        if prev is None:
            return True

        if isinstance(current, tuple):
            return any(self._values_changed(p, c) for p, c in zip(prev, current))

        if isinstance(current, list):
            if len(prev) != len(current):
                return True
            return any(self._values_changed(p, c) for p, c in zip(prev, current))

        if isinstance(current, Real) and not isinstance(current, bool):
            return abs(prev - current) > FLOAT_EPS

        return prev != current

    def _twist_state(self, msg: Twist):
        return (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )

    def _gait_params_state(self, msg: GaitParams):
        return (
            msg.standing,
            msg.stride_height,
            msg.t_st,
            msg.t_sw,
            tuple(msg.gait_offsets),
        )

    def _set_twist_state(self, msg: Twist, state):
        msg.linear.x = state[0]
        msg.linear.y = state[1]
        msg.linear.z = state[2]
        msg.angular.x = state[3]
        msg.angular.y = state[4]
        msg.angular.z = state[5]

    def _apply_cmd_pose_low_pass(self):
        target_state = self._twist_state(self.cmd_pose_msg)

        if self.filtered_cmd_pose_state is None:
            self.filtered_cmd_pose_state = target_state
        else:
            self.filtered_cmd_pose_state = tuple(
                prev + CMD_POSE_LPF_ALPHA * (target - prev)
                for prev, target in zip(self.filtered_cmd_pose_state, target_state)
            )

        self._set_twist_state(self.cmd_pose_msg, self.filtered_cmd_pose_state)

    def _publish_if_changed(
        self, publisher, msg, state_getter, prev_state_attr: str, force: bool = False
    ):
        current_state = state_getter(msg)
        prev_state = getattr(self, prev_state_attr)

        if force or self._values_changed(prev_state, current_state):
            publisher.publish(msg)
            setattr(self, prev_state_attr, current_state)

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < JOY_DEADZONE else value

    def _handle_joystick_control(self):
        standup_button = self.standup_button

        # --- STAND UP and LAY DOWN COMMANDS ---
        if standup_button == BUTTON_PRESSED and self.prev_standup_button == BUTTON_RELEASED:
            if self.is_sleeping:
                self.action_num = STANDUP
                self.send_action_request(self.action_num)
                self.is_sleeping = False
                self.robot_mode = self.mode_switch
            else:
                self.action_num = LAY_DOWN
                self.send_action_request(self.action_num)
                self.is_sleeping = True
                self.robot_mode = DO_NOTHING_MODE

        self.prev_standup_button = standup_button

        if self.is_sleeping:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pose_msg.linear.z = ROBOT_BODY_Z
            self.cmd_pose_msg.angular.x = 0.0
            self.cmd_pose_msg.angular.y = 0.0
            self.cmd_pose_msg.angular.z = 0.0
            self.gait_params_msg.standing = True
            return

        # --- SWITCH BETWEEN MODES ---
        self.robot_mode = self.mode_switch

        self.cmd_pose_msg.linear.z = ROBOT_BODY_Z
        self.cmd_pose_msg.angular.x = 0.0
        self.cmd_pose_msg.angular.y = 0.0
        self.cmd_pose_msg.angular.z = 0.0

        if self.robot_mode == LOCOMOTION_MODE:
            self.gait_params_msg.standing = False
            self.gait_params_msg.stride_height = self.stride_height
            self.gait_params_msg.t_st = self._t_st_for_speed(self.max_vel_x)
            self.gait_params_msg.t_sw = T_SW
            self.gait_params_msg.gait_offsets = GAIT_TYPE

            self.cmd_vel_msg.linear.x = -self._apply_deadzone(self.stick_x) * self.max_vel_x
            self.cmd_vel_msg.linear.y = (
                self._apply_deadzone(self.stick_y) * self.max_vel_x * LATERAL_SPEED_RATIO
            )
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = (
                self._apply_deadzone(self.stick_yaw) * self.max_vel_x * ANGULAR_SPEED_RATIO
            )
        elif self.robot_mode == BODY_MODE:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pose_msg.angular.x = self.stick_roll * BODY_ROLL_MAX
            self.cmd_pose_msg.angular.y = self.stick_pitch * BODY_PITCH_MAX
            self.cmd_pose_msg.angular.z = self.stick_yaw * BODY_YAW_MAX
            self.gait_params_msg.standing = True
        else:  # STANDING_MODE
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.x = 0.0
            self.cmd_vel_msg.angular.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.gait_params_msg.standing = True

    def timer_callback(self):
        self._handle_joystick_control()

        if self.prev_robot_mode != self.robot_mode:
            self.send_mode_request(self.robot_mode)

        if self.prev_robot_mode == DO_NOTHING_MODE and self.robot_mode != DO_NOTHING_MODE:
            self.cmd_pose_force_ticks = CMD_POSE_STAND_REPUBLISH_TICKS

        force_cmd_pose = self.cmd_pose_force_ticks > 0
        if force_cmd_pose:
            self.cmd_pose_force_ticks -= 1

        if self.is_sleeping:
            self.cmd_pose_sleep_ticks += 1
            if self.cmd_pose_sleep_ticks >= CMD_POSE_SLEEP_REPUBLISH_TICKS:
                self.cmd_pose_sleep_ticks = 0
                force_cmd_pose = True
        else:
            self.cmd_pose_sleep_ticks = 0

        if self.robot_mode != DO_NOTHING_MODE:
            self._apply_cmd_pose_low_pass()

        self._publish_if_changed(
            self.cmd_vel_pub,
            self.cmd_vel_msg,
            self._twist_state,
            "prev_cmd_vel_state",
        )
        self._publish_if_changed(
            self.cmd_pose_pub,
            self.cmd_pose_msg,
            self._twist_state,
            "prev_cmd_pose_state",
            force=force_cmd_pose,
        )
        self._publish_if_changed(
            self.gait_params_pub,
            self.gait_params_msg,
            self._gait_params_state,
            "prev_gait_params_state",
        )

        self.prev_robot_mode = self.robot_mode


def main(args=None):
    rclpy.init(args=args)

    radiolink_control = MorsRadiolinkControl()

    try:
        rclpy.spin(radiolink_control)
    finally:
        radiolink_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
