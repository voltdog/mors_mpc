import scipy.signal
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mors_ros_msgs.msg import GaitParams
from mors_ros_msgs.srv import RobotCmd
import scipy
import numpy as np

import sys
print(sys.executable)


DO_NOTHING_MODE = 0
EXPERIMENT_MODE = 1
STANDING_MODE = 3

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



STRIDE_HEIGHT = 0.05
T_SW = 0.2 #0.25 #
T_ST = 0.2 #0.35 #
GAIT_TYPE = [0.0, 0.5, 0.5, 0.0]

SWITCH_INTERVAL = 6.0
CYCLE_NUMBERS = 2.0

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
        self.gait_params_msg.stride_height = STRIDE_HEIGHT
        self.gait_params_msg.t_st = T_ST
        self.gait_params_msg.t_sw = T_SW
        self.gait_params_msg.gait_offsets = GAIT_TYPE

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ref_vel_x = CMD_VEL_X
        self.ref_vel_y = CMD_VEL_Y
        self.ref_vel_z = 0.0
        self.joy_ref_vel_x = 0.0
        self.joy_ref_vel_y = 0.0
        self.joy_ref_vel_z = 0.0

        self.ref_pose_z = 0.0
        self.ref_pose_roll = 0.0
        self.ref_pose_pitch = 0.0
        self.ref_pose_yaw = 0.0
        self.vel_coeff = CMD_VEL_X

        self.t_sw = T_SW
        self.t_st = T_ST
        self.stride_height = STRIDE_HEIGHT
        self.gait_offsets = GAIT_TYPE

        self.R1 = 0
        self.pre_R1 = 0
        self.R2 = 1
        self.pre_R2 = 0
        self.L2 = 0

        self.is_sleeping = True
        self.robot_mode = DO_NOTHING_MODE
        self.action_num = NO_ACTION

        self.cur_time = 0.0
        self.dt = timer_period
        self.pass_cnt = 0.0

        self.get_logger().info(f'Experiment 5: KMU Demonstration')

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

        # self.get_logger().info(f'L2: {self.L2}')

        self.joy_ref_vel_x = -msg.axes[2] * MAX_REF_VEL_X
        self.joy_ref_vel_y = 0.0 #msg.axes[3] * 0.2
        vel_z_coef = -2 * abs(self.joy_ref_vel_x) + 1.4
        self.joy_ref_vel_z = msg.axes[0] * vel_z_coef
        if abs(self.joy_ref_vel_z) < 0.1:
            self.joy_ref_vel_z = 0.0

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
            # self.get_logger().info("Clicked!")
            self.action_num = STANDUP
            self.send_action_request(self.action_num)

            if self.R2_cur == 0:
                self.robot_mode = EXPERIMENT_MODE
            elif self.R2_cur == 1:
                self.robot_mode = STANDING_MODE

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

        # --- SWITCH MODES BETWEEN STANDING AND EXPERIMENTAL ---
        if self.is_sleeping == False and self.pre_R2 != 0 and self.R2_cur == 0:
            self.robot_mode = EXPERIMENT_MODE
            self.send_mode_request(self.robot_mode)
        elif self.is_sleeping == False and self.pre_R2 != 1 and self.R2_cur == 1:
            self.robot_mode = STANDING_MODE
            self.cur_time = 0.0
            self.send_mode_request(self.robot_mode)

        # --- CONDUCTING THE EXPERIMENT --- 
        if self.is_sleeping == False and self.robot_mode == EXPERIMENT_MODE:
            if self.L2 == 0:
                if self.cur_time < 1.0:
                    self.ref_vel_x = 0.0
                    self.ref_vel_y = 0.0
                elif 1.0 < self.cur_time < SWITCH_INTERVAL+1:
                    self.ref_vel_x = CMD_VEL_X
                    self.ref_vel_y = 0.0
                elif SWITCH_INTERVAL+1 < self.cur_time < SWITCH_INTERVAL+2:
                    self.ref_vel_x = 0.0
                    self.ref_vel_y = 0.0
                elif SWITCH_INTERVAL+2 < self.cur_time < 2*SWITCH_INTERVAL+2:
                    self.ref_vel_x = -CMD_VEL_X
                    self.ref_vel_y = 0.0
                elif 2*SWITCH_INTERVAL+2 < self.cur_time < 2*SWITCH_INTERVAL+3:
                    self.ref_vel_x = 0.0
                    self.ref_vel_y = 0.0
                else:
                    # if self.pass_cnt < CYCLE_NUMBERS:
                    self.pass_cnt += 1
                    self.cur_time = 0.0
                    self.get_logger().info(f'Cycle Number: {self.pass_cnt}')
                    # else:
                    #     self.robot_mode = STANDING_MODE
                    #     self.send_mode_request(self.robot_mode)
                self.cmd_vel_msg.linear.x = self.ref_vel_x 
                self.cmd_vel_msg.linear.y = self.ref_vel_y
                self.cmd_vel_msg.angular.z = self.ref_vel_z

                self.cur_time += self.dt
            else:
                self.cmd_vel_msg.linear.x = self.joy_ref_vel_x 
                self.cmd_vel_msg.linear.y = self.joy_ref_vel_y
                self.cmd_vel_msg.angular.z = self.joy_ref_vel_z

            self.gait_params_msg.standing = False
        else:
            self.cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_msg.linear.y = 0.0
            self.cmd_vel_msg.angular.z = 0.0
            self.cmd_pose_msg.angular.x = 0.0
            self.cmd_pose_msg.angular.y = 0.0
            self.cmd_pose_msg.angular.z = 0.0
            self.gait_params_msg.standing = True
            # self.cur_time = 0.0

        # --- PUBLISH THE DATA ---
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.cmd_pose_pub.publish(self.cmd_pose_msg)
        self.gait_params_pub.publish(self.gait_params_msg)

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
