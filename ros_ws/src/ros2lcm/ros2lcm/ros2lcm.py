import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64MultiArray
import lcm

from pathlib import Path
import sys
for parent in Path(__file__).resolve().parents:
    if (parent / "lcm_msgs").is_dir():
        sys.path.insert(0, str(parent / "lcm_msgs"))
        break

from mors_msgs.leg_cmd_msg import leg_cmd_msg

LEG_CMD_CHANNEL = "LEG_CMD"
SERVO_STATE_CHANNEL = "SERVO_STATE"
SERVO_CMD_CHANNEL = "SERVO_CMD"
IMU_CHANNEL = "IMU_DATA"

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('ros2lcm_leg')
        self.grf_subscription = self.create_subscription(
            Float64MultiArray,
            'leg_cmd_grf',
            self.leg_grf_callback,
            10)
        self.pos_subscription = self.create_subscription(
            Float64MultiArray,
            'leg_cmd_pos',
            self.leg_pos_callback,
            10)
        self.vel_subscription = self.create_subscription(
            Float64MultiArray,
            'leg_cmd_vel',
            self.leg_vel_callback,
            10)
        self.grf_subscription  # prevent unused variable warning
        self.pos_subscription
        self.vel_subscription

        self.lc = lcm.LCM()
        self.lcm_msg = leg_cmd_msg()
        

    def leg_grf_callback(self, msg : Float64MultiArray):
        # self.get_logger().info('I heard: "%s"' % msg.data[11])
        for i in range(3):
            self.lcm_msg.r1_grf[i] = msg.data[i]
            self.lcm_msg.l1_grf[i] = msg.data[i+3]
            self.lcm_msg.r2_grf[i] = msg.data[i+6]
            self.lcm_msg.l2_grf[i] = msg.data[i+9]
        self.lc.publish(LEG_CMD_CHANNEL, self.lcm_msg.encode())

    def leg_pos_callback(self, msg : Float64MultiArray):
        # self.get_logger().info('I heard: "%s"' % msg.data[11])
        for i in range(3):
            self.lcm_msg.r1_pos[i] = msg.data[i]
            self.lcm_msg.l1_pos[i] = msg.data[i+3]
            self.lcm_msg.r2_pos[i] = msg.data[i+6]
            self.lcm_msg.l2_pos[i] = msg.data[i+9]
        self.lc.publish(LEG_CMD_CHANNEL, self.lcm_msg.encode())

    def leg_vel_callback(self, msg : Float64MultiArray):
        # self.get_logger().info('I heard: "%s"' % msg.data[11])
        for i in range(3):
            self.lcm_msg.r1_vel[i] = msg.data[i]
            self.lcm_msg.l1_vel[i] = msg.data[i+3]
            self.lcm_msg.r2_vel[i] = msg.data[i+6]
            self.lcm_msg.l2_vel[i] = msg.data[i+9]
        self.lc.publish(LEG_CMD_CHANNEL, self.lcm_msg.encode())
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
