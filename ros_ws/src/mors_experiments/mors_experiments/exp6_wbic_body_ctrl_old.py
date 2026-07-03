import scipy.signal
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
import scipy
import numpy as np
from numbers import Real

import sys
print(sys.executable)


DO_NOTHING_MODE = 0
LOCOMOTION_MODE = 1
BODY_MODE = 2
STANDING_MODE = 3

SLOW_MODE = 0
MIDDLE_MODE = 1
FAST_MODE = 2

NO_ACTION = 0
STANDUP = 1
LAY_DOWN = 2

SLEEPING = 0
IDLE = 1

CMD_VEL_X = 0.0
CMD_VEL_Y = 0.0
# MAX_ROBOT_HEIGHT_OFFSET = 0.02
MAX_REF_VEL_X = 0.6
MAX_REF_VEL_Z = 1.4
# MAX_REF_ROLL = 0.3
# MAX_REF_PITCH = 0.3
# MAX_REF_YAW = 0.3

BODY_Z_MAX = 0.25
BODY_Z_MIN = 0.1
BODY_ROLL_MAX = 0.707
BODY_PITCH_MAX = 0.707
BODY_YAW_MAX = 0.707

STRIDE_HEIGHT_SLOW = 0.05
T_SW_SLOW = 0.25 
T_ST_SLOW = 0.75 
GAIT_TYPE_SLOW = [0.0, 0.25, 0.5, 0.75]

STRIDE_HEIGHT_MIDDLE = 0.05
T_SW_MIDDLE = 0.25 
T_ST_MIDDLE = 0.35 
GAIT_TYPE_MIDDLE = [0.0, 0.5, 0.5, 0.0]

STRIDE_HEIGHT_FAST = 0.05
T_SW_FAST = 0.2 
T_ST_FAST = 0.15 
GAIT_TYPE_FAST = [0.0, 0.5, 0.5, 0.0]

SWITCH_INTERVAL = 6.0
CYCLE_NUMBERS = 2.0
FLOAT_EPS = 1e-6

class RadiolinkTeleop(Node):
    def __init__(self):
        super().__init__("radiolink_teleop")

        self.get_logger().info(f'Python executable: {sys.executable}')

        self.mode_cli = self.create_client(RobotCmd, "robot_mode")
        self.req = RobotCmd.Request()

        self.action_cli = self.create_client(RobotCmd, "robot_action")
        # self.action_req = RobotCmd.Request()

        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.cmd_vel_msg = Twist()
        self.cmd_pose_pub = self.create_publisher(Twist, "cmd_pose", 10)
        self.cmd_pose_msg = Twist()
        self.gait_params_pub = self.create_publisher(GaitParams, "gait_params", 10)
        self.gait_params_msg = GaitParams()
        self.gait_params_msg.standing = True
        self.gait_params_msg.stride_height = STRIDE_HEIGHT_MIDDLE
        self.gait_params_msg.t_st = T_ST_MIDDLE
        self.gait_params_msg.t_sw = T_SW_MIDDLE
        self.gait_params_msg.gait_offsets = GAIT_TYPE_MIDDLE

        self.prev_cmd_vel_state = None
        self.prev_cmd_pose_state = None
        self.prev_gait_params_state = None

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ref_vel_x = CMD_VEL_X
        self.ref_vel_y = CMD_VEL_Y
        self.ref_vel_z = 0.0
        self.joy_R3_x = 0.0
        self.joy_ref_vel_y = 0.0
        self.joy_R3_y = 0.0

        self.ref_pose_z = 0.0
        self.ref_pose_roll = 0.0
        self.ref_pose_pitch = 0.0
        self.ref_pose_yaw = 0.0
        self.vel_coeff = CMD_VEL_X

        self.t_sw = T_SW_MIDDLE
        self.t_st = T_ST_MIDDLE
        self.stride_height = STRIDE_HEIGHT_MIDDLE
        self.gait_offsets = GAIT_TYPE_MIDDLE

        self.R1 = 0
        self.pre_R1 = 0
        self.R2 = 1
        self.pre_R2 = 0
        self.L2 = 0
        self.joy_L1 = 0

        self.is_sleeping = True
        self.robot_mode = DO_NOTHING_MODE
        self.action_num = NO_ACTION

        self.cur_time = 0.0
        self.dt = timer_period
        self.pass_cnt = 0.0

        self.get_logger().info(f'Experiment 6: WBC Tests')

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

    def _publish_if_changed(self, publisher, msg, state_getter, prev_state_attr: str):
        current_state = state_getter(msg)
        prev_state = getattr(self, prev_state_attr)

        if self._values_changed(prev_state, current_state):
            publisher.publish(msg)
            setattr(self, prev_state_attr, current_state)

    def joy_callback(self, msg: Joy):
        # self.get_logger().info('Hello')

        if msg.axes[5] > 0.5:
            self.R1 = 0
        elif msg.axes[5] < -0.5:
            self.R1 = 1

        if msg.axes[4] > 0.5:
            self.R2 = 0
        elif abs(msg.axes[4]) < 0.5:
            self.R2 = 1
        elif msg.axes[4] < -0.5:
            self.R2 = 2

        if msg.axes[6] > 0.5:
            self.L2 = 0
        elif abs(msg.axes[6]) < 0.5:
            self.L2 = 1
        elif msg.axes[6] < -0.5:
            self.L2 = 2

        self.joy_R3_x = -msg.axes[2]
        self.joy_R3_y = msg.axes[0]
        self.joy_L3_x = msg.axes[1]
        self.joy_L3_y = msg.axes[3]

        self.joy_L1 = msg.axes[7]

        # self.get_logger().info(f'R1: {self.R1}')

    def send_mode_request(self, mode: int):
        # self.get_logger().info(f'mode: {mode}')
        self.req.data = mode
        resp = self.mode_cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        return resp.result()
    
    def send_action_request(self, mode: int):
        # self.get_logger().info(f'mode: {mode}')
        self.req.data = mode
        resp = self.action_cli.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        return resp.result()

    def timer_callback(self):
        # self.get_logger().info('X: "%s"' % self.ref_vel_x)
        self.R1_cur = self.R1
        self.R2_cur = self.R2

        # --- STAND UP and LAY DOWN COMMANDS ---
        if self.R1_cur == 1 and self.pre_R1 == 0 and self.is_sleeping == True:
            self.get_logger().info("Clicked!")
            self.action_num = STANDUP
            
            self.send_action_request(self.action_num)

            self.get_logger().info("Yo!")

            if self.R2_cur == 0:
                self.robot_mode = LOCOMOTION_MODE
            elif self.R2_cur == 1:
                self.robot_mode = STANDING_MODE
            elif self.R2_cur == 2:
                self.robot_mode = BODY_MODE

            self.send_mode_request(self.robot_mode)
            self.is_sleeping = False
            self.get_logger().info(f'is_sleeping: {self.is_sleeping}')
        elif self.R1_cur == 1 and self.pre_R1 == 0 and self.is_sleeping == False:
            # self.get_logger().info("Clicked!")
            self.action_num = LAY_DOWN
            self.send_action_request(self.action_num)
            self.is_sleeping = True
            self.robot_mode = DO_NOTHING_MODE
            self.send_mode_request(self.robot_mode)
            self.get_logger().info(f'is_sleeping: {self.is_sleeping}')

        # --- SWITCH BETWEEN MODES ---
        if self.is_sleeping == False and self.R2_cur == 0:
            self.robot_mode = LOCOMOTION_MODE
        elif self.is_sleeping == False and self.R2_cur == 1:
            self.robot_mode = STANDING_MODE
        elif self.is_sleeping == False and self.R2_cur == 2:
            self.robot_mode = BODY_MODE

        if self.is_sleeping == False and self.L2_cur == 0:
            self.gait_mode = FAST_MODE
        elif self.is_sleeping == False and self.L2_cur == 1:
            self.gait_mode = MIDDLE_MODE
        elif self.is_sleeping == False and self.L2_cur == 2:
            self.gait_mode = SLOW_MODE

        # --- SWITCH BETWEEN DIFFERENT GAITS ---
        # if self.is_sleeping == False and self.pre_R2 != 0 and self.R2_cur == 0:
        #     self.robot_mode = LOCOMOTION_MODE
        #     self.send_mode_request(self.robot_mode)
        # elif self.is_sleeping == False and self.pre_R2 != 1 and self.R2_cur == 1:
        #     self.robot_mode = STANDING_MODE
        #     self.cur_time = 0.0
        #     self.send_mode_request(self.robot_mode)
        if self.is_sleeping == False:
            cmd_pos_z1 = BODY_Z_MIN + (self.joy_L1 + 1) * (BODY_Z_MAX - BODY_Z_MIN) / 2 
            self.cmd_pose_msg.linear.z = cmd_pos_z1

            if self.robot_mode == LOCOMOTION_MODE:
                self.cmd_vel_msg.linear.x = self.joy_L3_x 
                self.cmd_vel_msg.linear.y = self.joy_L3_y
                self.cmd_vel_msg.angular.z = self.joy_R3_y
                self.gait_params_msg.standing = False
                if self.gait_mode == SLOW_MODE:
                    self.gait_params_msg.stride_height = STRIDE_HEIGHT_SLOW
                    self.gait_params_msg.t_st = T_ST_SLOW
                    self.gait_params_msg.t_sw = T_SW_SLOW
                    self.gait_params_msg.gait_offsets = GAIT_TYPE_SLOW
                elif self.gait_mode == MIDDLE_MODE:
                    self.gait_params_msg.stride_height = STRIDE_HEIGHT_MIDDLE
                    self.gait_params_msg.t_st = T_ST_MIDDLE
                    self.gait_params_msg.t_sw = T_SW_MIDDLE
                    self.gait_params_msg.gait_offsets = GAIT_TYPE_MIDDLE
                elif self.gait_mode == FAST_MODE:
                    self.gait_params_msg.stride_height = STRIDE_HEIGHT_FAST
                    self.gait_params_msg.t_st = T_ST_FAST
                    self.gait_params_msg.t_sw = T_SW_FAST
                    self.gait_params_msg.gait_offsets = GAIT_TYPE_FAST
                
            elif self.robot_mode == DO_NOTHING_MODE:
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_msg.linear.y = 0.0
                self.cmd_vel_msg.linear.z = 0.0
                self.cmd_pose_msg.angular.x = 0.0
                self.cmd_pose_msg.angular.y = 0.0
                self.cmd_pose_msg.angular.z = 0.0
                self.gait_params_msg.standing = True
            elif self.robot_mode == BODY_MODE:
                cmd_pos_z = cmd_pos_z1 + self.joy_L3_x * (BODY_Z_MAX + BODY_Z_MIN)/2.0
                cmd_pos_z = np.clip(cmd_pos_z, BODY_Z_MIN, BODY_Z_MAX)
                self.cmd_pose_msg.linear.z = cmd_pos_z
                self.cmd_pose_msg.angular.x = self.joy_R3_y * BODY_ROLL_MAX
                self.cmd_pose_msg.angular.y = self.joy_R3_x * BODY_PITCH_MAX
                self.cmd_pose_msg.angular.z = self.joy_L3_y * BODY_YAW_MAX
                self.gait_params_msg.standing = True
                # self.get_logger().info(f'{cmd_pos_z}')


        # --- PUBLISH THE DATA ---
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
        )
        self._publish_if_changed(
            self.gait_params_pub,
            self.gait_params_msg,
            self._gait_params_state,
            "prev_gait_params_state",
        )

        

        # --- SAVE PREVIOUS STATES ---
        self.pre_R1 = self.R1_cur
        self.pre_R2 = self.R2_cur
        


def main(args=None):
    rclpy.init(args=args)

    radiolink = RadiolinkTeleop()

    rclpy.spin(radiolink)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    radiolink.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
