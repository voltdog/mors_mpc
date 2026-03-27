import scipy.signal
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
import scipy
import numpy as np


DO_NOTHING_MODE = 0
LOCOMOTION_MODE = 1
BODY_CONTROL_MODE = 2
STANDING_MODE = 3

NO_ACTION = 0
STANDUP = 1
LAY_DOWN = 2
GIVE_HAND = 3

SLEEPING = 0
IDLE = 1

MAX_ROBOT_HEIGHT_OFFSET = 0.02
MAX_REF_VEL_Z = 0.6
MAX_REF_ROLL = 0.3
MAX_REF_PITCH = 0.3
MAX_REF_YAW = 0.3

VEL_LOW = 0.4
STRIDE_HEIGHT_LOW = 0.08
T_SW_LOW = 0.21
T_ST_LOW = 0.3
GAIT_TYPE_LOW = [0.0, 0.5, 0.5, 0.0]

# # FAST TROT
# VEL_NORMAL = 0.2
# STRIDE_HEIGHT_NORMAL = 0.08
# T_SW_NORMAL = 0.21
# T_ST_NORMAL = 0.22
# GAIT_TYPE_NORMAL = [0.0, 0.5, 0.5, 0.0]

# WALK
VEL_NORMAL = 0.2
STRIDE_HEIGHT_NORMAL = 0.08
T_SW_NORMAL = 0.21
T_ST_NORMAL = 0.9
GAIT_TYPE_NORMAL = [0.0, 0.25, 0.5, 0.75]

# PRONK
# VEL_NORMAL = 0.2
# STRIDE_HEIGHT_NORMAL = 0.05
# T_SW_NORMAL = 0.21
# T_ST_NORMAL = 0.2
# GAIT_TYPE_NORMAL = [0.0, 0.0, 0.0, 0.0]

# PACE
VEL_HIGH = 0.2
STRIDE_HEIGHT_HIGH = 0.06
T_SW_HIGH = 0.21
T_ST_HIGH = 0.45
GAIT_TYPE_HIGH = [0.0, 0.44, 0.12, 0.56]

# BOUND
# VEL_HIGH = 0.2
# STRIDE_HEIGHT_HIGH = 0.07
# T_SW_HIGH = 0.21
# T_ST_HIGH = 0.4
# GAIT_TYPE_HIGH = [0.0, 0.15, 0.4, 0.55]

# GALLOP
# VEL_HIGH = 0.2
# STRIDE_HEIGHT_HIGH = 0.06
# T_SW_HIGH = 0.21
# T_ST_HIGH = 0.4
# GAIT_TYPE_HIGH = [0.0, 0.8571, 0.3571, 0.5]

VEL_THRESHOLD = [0.05, 0.05, 0.05]

LOW = 0
NORMAL = 1
HIGH = 2

class RadiolinkTeleop(Node):
    def __init__(self):
        super().__init__("radiolink_teleop")

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

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ref_vel_x = 0.0
        self.ref_vel_y = 0.0
        self.ref_vel_z = 0.0

        self.ref_pose_z = 0.0
        self.ref_pose_roll = 0.0
        self.ref_pose_pitch = 0.0
        self.ref_pose_yaw = 0.0
        self.vel_coeff = VEL_LOW

        self.t_sw = 0.3
        self.t_st = 0.3
        self.stride_height = 0.06
        self.gait_offsets = [0.0, 0.5, 0.5, 0.0]

        self.R1 = 0
        self.pre_R1 = 0
        self.R2 = 1
        self.pre_R2 = 0
        self.L2 = 0

        self.is_sleeping = True
        self.robot_mode = DO_NOTHING_MODE
        self.action_num = NO_ACTION
        

        self.vel_thresholds = VEL_THRESHOLD

        self.b_lpf_x, self.a_lpf_x = scipy.signal.iirfilter(
            1, Wn=24.0, fs=1/timer_period, btype="low", ftype="butter"
        )
        self.z_lpf_x = np.zeros(self.b_lpf_x.size - 1)

        self.b_lpf_y, self.a_lpf_y = scipy.signal.iirfilter(
            1, Wn=24.0, fs=1/timer_period, btype="low", ftype="butter"
        )
        self.z_lpf_y = np.zeros(self.b_lpf_y.size - 1)

        self.b_lpf_gait, self.a_lpf_gait = scipy.signal.iirfilter(
            1, Wn=1.0, fs=1/timer_period, btype="low", ftype="butter"
        )
        self.z_lpf_gait = np.zeros(self.b_lpf_gait.size - 1)

    def joy_callback(self, msg: Joy):
        # self.get_logger().info('Hello')
        self.ref_vel_x = -msg.axes[2]
        self.ref_vel_y = msg.axes[3]
        # if abs(self.ref_vel_x) < 0.15:
        #     self.ref_vel_x = 0.0
        # self.get_logger().info(f"{self.ref_vel_x}")

        self.ref_vel_z = msg.axes[0] * MAX_REF_VEL_Z
        if abs(self.ref_vel_z) < 0.1:
            self.ref_vel_z = 0.0
        self.ref_pose_z = msg.axes[1] * MAX_ROBOT_HEIGHT_OFFSET
        if abs(self.ref_pose_z) < 0.002:
            self.ref_pose_z = 0.0
        self.ref_pose_roll = msg.axes[3] * MAX_REF_ROLL
        self.ref_pose_pitch = -msg.axes[2] * MAX_REF_PITCH
        self.ref_pose_yaw = msg.axes[0] * MAX_REF_YAW

        if msg.axes[5] > 0.5:
            self.R1 = 0
        elif msg.axes[5] < -0.5:
            self.R1 = 1

        if msg.axes[6] > 0.5:
            self.locomode = HIGH
            self.vel_coeff = VEL_HIGH
            self.gait_params_msg.stride_height = STRIDE_HEIGHT_HIGH
            self.gait_params_msg.t_st = T_ST_HIGH
            self.gait_params_msg.t_sw = T_SW_HIGH
            self.gait_params_msg.gait_offsets = GAIT_TYPE_HIGH
            
        elif abs(msg.axes[6]) < 0.5:
            self.locomode = NORMAL
            self.vel_coeff = VEL_NORMAL
            self.gait_params_msg.stride_height = STRIDE_HEIGHT_NORMAL
            self.gait_params_msg.t_st = T_ST_NORMAL
            self.gait_params_msg.t_sw = T_SW_NORMAL
            self.gait_params_msg.gait_offsets = GAIT_TYPE_NORMAL

        elif msg.axes[6] < -0.5:
            self.locomode = LOW
            self.vel_coeff = VEL_LOW
            self.gait_params_msg.stride_height = STRIDE_HEIGHT_LOW
            self.gait_params_msg.t_st = T_ST_LOW
            self.gait_params_msg.t_sw = T_SW_LOW
            self.gait_params_msg.gait_offsets = GAIT_TYPE_LOW

        if msg.axes[4] > 0.5:
            self.R2 = 0
        elif abs(msg.axes[4]) < 0.5:
            self.R2 = 1
        elif msg.axes[4] < -0.5:
            self.R2 = 2

        # self.get_logger().info(f'R1: {self.R1}')

    def send_mode_request(self, mode: int):
        self.get_logger().info(f'rediolink: mode: {mode}')

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

        if self.R1_cur == 1 and self.pre_R1 == 0 and self.is_sleeping == True:
            # self.get_logger().info("Clicked!")
            self.action_num = STANDUP
            self.send_action_request(self.action_num)

            if self.R2_cur == 0:
                self.robot_mode = LOCOMOTION_MODE
            elif self.R2_cur == 1:
                self.robot_mode = STANDING_MODE
            elif self.R2_cur == 2:
                self.robot_mode = BODY_CONTROL_MODE

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

        if self.is_sleeping == False and self.pre_R2 != 0 and self.R2_cur == 0:
            self.robot_mode = LOCOMOTION_MODE
            self.send_mode_request(self.robot_mode)
        elif self.is_sleeping == False and self.pre_R2 != 1 and self.R2_cur == 1:
            self.robot_mode = STANDING_MODE
            self.send_mode_request(self.robot_mode)
        elif self.is_sleeping == False and self.pre_R2 != 2 and self.R2_cur == 2:
            self.robot_mode = BODY_CONTROL_MODE
            self.send_mode_request(self.robot_mode)

        if self.is_sleeping == False and self.R2 == 0:
            # res_x, self.z_lpf_x = scipy.signal.lfilter(
            #     self.b_lpf_x, self.a_lpf_x, [self.ref_vel_x], zi=self.z_lpf_x
            # )
            # res_y, self.z_lpf_y = scipy.signal.lfilter(
            #     self.b_lpf_y, self.a_lpf_y, [self.ref_vel_y], zi=self.z_lpf_y
            # )

            # if abs(ref_vel_x_) < (0.15 * self.vel_coeff) or abs(res[0]) < (
            #     0.15 * self.vel_coeff
            # ):
            #     self.cmd_vel_msg.linear.x = 0.0
            # else:
            #     if ref_vel_x_ > 0:
            #         self.cmd_vel_msg.linear.x = res[0] - (0.2 * self.vel_coeff)
            #     else:
            #         self.cmd_vel_msg.linear.x = res[0] + (0.2 * self.vel_coeff)
            
            # if np.abs(res_x[0]) < 0.1:
            #     res_x[0] = 0.0
            # if np.abs(res_y[0]) < 0.1:
            #     res_y[0] = 0.0
            # self.cmd_vel_msg.linear.x = res_x[0] * self.vel_coeff
            # self.cmd_vel_msg.linear.y = res_y[0] * self.vel_coeff/2

            if np.abs(self.ref_vel_x * self.vel_coeff) < self.vel_thresholds[self.locomode]:
                self.ref_vel_x = 0.0
            if np.abs(self.ref_vel_y * self.vel_coeff/2) < self.vel_thresholds[self.locomode]:
                self.ref_vel_y = 0.0
            self.cmd_vel_msg.linear.x = self.ref_vel_x * self.vel_coeff
            self.cmd_vel_msg.linear.y = self.ref_vel_y * self.vel_coeff/2

            self.cmd_vel_msg.angular.z = self.ref_vel_z
            self.cmd_pose_msg.linear.z = self.ref_pose_z

            self.gait_params_msg.standing = False
        elif self.is_sleeping == False and self.R2 == 2:
            self.cmd_pose_msg.linear.z = self.ref_pose_z
            self.cmd_pose_msg.angular.x = self.ref_pose_roll
            self.cmd_pose_msg.angular.y = self.ref_pose_pitch
            self.cmd_pose_msg.angular.z = self.ref_pose_yaw
            self.gait_params_msg.standing = True
        else:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pose_msg.angular.x = 0.0
            self.cmd_pose_msg.angular.y = 0.0
            self.cmd_pose_msg.angular.z = 0.0
            self.gait_params_msg.standing = True

        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.cmd_pose_pub.publish(self.cmd_pose_msg)
        self.gait_params_pub.publish(self.gait_params_msg)

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
