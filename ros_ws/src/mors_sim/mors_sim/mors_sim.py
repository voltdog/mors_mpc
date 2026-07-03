import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from cgitb import reset
from sys import flags
import pybullet as p
import time
from threading import Thread
import pybullet_data
import lcm

from mors_msgs.servo_cmd_msg import servo_cmd_msg
from mors_msgs.servo_state_msg import servo_state_msg 
from mors_msgs.imu_lcm_data import imu_lcm_data
from mors_msgs.robot_state_msg import robot_state_msg
from mors_msgs.contact_sensor_msg import contact_sensor_msg
from mors_msgs.odometry_msg import odometry_msg

from additional.mors_gym_env import MorsBulletEnv
from additional.forward_kinematics import ForwardKinematics

import configparser
import numpy as np
# import rospy
# import tf2_ros
from PIL import Image as pil

from whole_body_state_msgs.msg import WholeBodyState
from whole_body_state_msgs.msg import JointState as WBJointState
from whole_body_state_msgs.msg import ContactState as WBContactState
from sensor_msgs.msg import Image, Imu, JointState, PointCloud2, PointField, LaserScan, CameraInfo
from whole_body_state_msgs.msg import Contacts
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist, Point
from rosgraph_msgs.msg import Clock

import rclpy
from rclpy.node import Node
import tf2_ros


class Hardware_Level_Sim(Node):
    def __init__(self):
        super().__init__('mors_pybullet')

        self.ref_pos = [0]*12
        self.ref_data = [0]*12
        self.init_ref_pos = [0]*12
        self.ref_vel = [0]*12
        self.ref_torq = [0]*12
        self.kp = [0]*12
        self.kd = [0]*12
        self.joint_dir = [-1]*12
        self.joint_offset = [0]*12
        self.theta_ref = [0]*12

        self.read_config()

        # init ROS publishers
        self.imu_data_pub = self.create_publisher(Imu, self.ros_imu_topic, 10) # #
        if self.foot_contacts_enabled:
            self.contact_pub = self.create_publisher(Contacts, self.ros_contact_flags_topic, 10) # #

        if self.ros_whole_body_state_enabled:
            self.wbc_state_pub = self.create_publisher(WholeBodyState, self.ros_whole_body_state_topic, 10) # #
            self.odom_pub = self.create_publisher(Odometry, self.ros_robot_odom_topic, 10) #
            self.vel_pub = self.create_publisher(Point, "robot_velocity", 10)

        if self.camera_enabled:
            self.rgb_image_pub = self.create_publisher(Image, self.camera_rgb_topic, 10) # #
            self.rgb_info_pub = self.create_publisher(CameraInfo, self.camera_rgb_info_topic, 10) # #
            self.depth_image_pub = self.create_publisher(Image, self.camera_depth_image_topic, 10) # #
            self.depth_info_pub = self.create_publisher(CameraInfo, self.camera_depth_info_topic, 10) # #
            if self.pointcloud_enabled:
                self.depth_points_pub = self.create_publisher(PointCloud2, self.camera_depth_points_topic, 10) #

        if self.lidar_enabled:
            self.lidar_pub = self.create_publisher(LaserScan, self.lidar_topic, 10) #

        self.js_pub = self.create_publisher(JointState, self.ros_joint_states_topic, 30) # #
        

        # because of SLAM
        if self.ros_whole_body_state_enabled:
            self.robot_tf = tf2_ros.TransformBroadcaster(self) #
        self.clock_pub = self.create_publisher(Clock, self.clock_topic, 10)

        # init ROS messages
        self.imu_msg = Imu()
        self.js_msg = JointState()
        self.wbs = WholeBodyState()
        self.odom = Odometry()
        self.img_rgb_msg = Image()
        self.img_depth_msg = Image()
        self.contact_msg = Contacts()
        self.lidar_msg = LaserScan()
        self.pointcloud_msg = PointCloud2()
        self.robot_vel_msg = Point()
        self.clock_msg = Clock()
        # self.clock_msg.clock.secs = self.get_clock().now().to_sec()
        self.zero_time = self.get_clock().now().to_msg()

        if self.camera_enabled:
            self.cam_info_msg = CameraInfo()
            self.cam_info_msg.header.stamp = self.get_clock().now()
            self.cam_info_msg.header.frame_id = self.camera_frame
            self.cam_info_msg.height = self.pixel_height
            self.cam_info_msg.width = self.pixel_width
            self.cam_info_msg.distortion_model = "plumb_bob"
            self.cam_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.cam_info_msg.k = [524.2422531097977, 0.0, 320.5, 0.0, 524.2422531097977, 240.5, 0.0, 0.0, 1.0]
            self.cam_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.cam_info_msg.p = [524.2422531097977, 0.0, 320.5, -0.0, 0.0, 524.2422531097977, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.cam_info_msg.binning_x = 0
            self.cam_info_msg.binning_y = 0
            self.cam_info_msg.roi.x_offset = 0
            self.cam_info_msg.roi.y_offset = 0
            self.cam_info_msg.roi.height = 0
            self.cam_info_msg.roi.width = 0
            self.cam_info_msg.roi.do_rectify = False

            self.img_rgb_msg.header.frame_id = self.camera_frame
            self.img_rgb_msg.width = self.pixel_width
            self.img_rgb_msg.height = self.pixel_height
            self.img_rgb_msg.encoding = "rgb8"
            self.img_rgb_msg.step = self.pixel_width

            self.img_depth_msg.header.frame_id = self.camera_frame
            self.img_depth_msg.width = self.pixel_width
            self.img_depth_msg.height = self.pixel_height
            self.img_depth_msg.encoding = "32FC1"
            self.img_depth_msg.step = self.pixel_width

        if self.lidar_enabled:
            self.lidar_msg.header.frame_id = self.lidar_frame
            self.lidar_msg.angle_min = self.lidar_angle_min
            self.lidar_msg.angle_max = self.lidar_angle_max
            self.lidar_msg.angle_increment = (np.abs(self.lidar_angle_min) + np.abs(self.lidar_angle_max)) / self.lidar_point_num#self.lidar_angle_increment
            self.lidar_msg.scan_time = 1/self.lidar_freq
            self.lidar_msg.time_increment = self.lidar_time_increment
            self.lidar_msg.range_min = self.lidar_range_min
            self.lidar_msg.range_max = self.lidar_range_max
            self.lidar_msg.intensities = [0]*self.lidar_point_num

        self.imu_msg.orientation_covariance = [2.603e-07, 0.0, 0.0, 0.0, 2.603e-07, 0.0, 0.0, 0.0, 0.0]
        self.imu_msg.angular_velocity_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        self.imu_msg.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]

        # init LCM thread
        self.cmd_th = Thread(target=self.get_cmd, args=())
        self.cmd_th.daemon = True
        self.cmd_th.start()

        # init LCM for states
        self.servo_state_msg = servo_state_msg()
        self.lcm_imu_msg = imu_lcm_data()
        self.lcm_robot_state_msg = robot_state_msg()
        self.lcm_odom_msg = odometry_msg()
        self.lcm_contact_sensor_msg = contact_sensor_msg()

        self.lc_servo_state = lcm.LCM()
        self.lc_imu = lcm.LCM()
        self.lc_robot_state = lcm.LCM()
        self.lc_odom = lcm.LCM()
        self.lc_contact = lcm.LCM()

        self.lcm_imu_msg.orientation_covariance = [2.603e-07, 0.0, 0.0, 0.0, 2.603e-07, 0.0, 0.0, 0.0, 0.0]
        self.lcm_imu_msg.angular_velocity_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        self.lcm_imu_msg.linear_acceleration_covariance = [2.5e-05, 0.0, 0.0, 0.0, 2.5e-05, 0.0, 0.0, 0.0, 2.5e-05]
        
        self.sim_it = 0
        self.init_simulation()
        self.set_start_position()

        if self.camera_enabled:
            cam_th = Thread(target=self.camera_loop, args=())
            cam_th.daemon = True
            cam_th.start()

        self.body_quaternion = [0,0,0,1]
        self.body_lin_pos = [0]*3
        self.body_ang_vel = [0]*3
        self.body_lin_vel = np.array([0]*3, float)
        self.body_lin_vel_prev = np.array([0]*3, float)
        self.body_lin_acc = np.array([0]*3, float)
        self.force_dir = 1

        self.imu_data = [0]*13

        self.motor_data = {}
        self.motor_data["pos"] = [0] * 12
        self.motor_data["vel"] = [0] * 12
        self.motor_data["torq"] = [0] * 12
        self.motor_data["name"] = [""] * 12

        self.leg_data_transform = [ 1,  1,  1, 
                                    1,  1,  1,
                                    1,  1,  1,
                                    1,  1,  1]
        # self.leg_data_order = [6, 8, 7, 9, 11, 10, 0, 2, 1, 3, 5, 4]
        self.leg_data_order = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

        self.fkine = ForwardKinematics()

        self.timer = self.create_timer(self.sim_period, self.loop)

    def read_config(self):

        self.declare_parameter('frequency', 500)
        self.declare_parameter('urdf_root', "./mors_sim/urdf")
        self.declare_parameter('world', "empty")
        self.declare_parameter('render', True)
        self.declare_parameter('on_rack', True)
        self.declare_parameter('foot_contacts', True)
        self.declare_parameter('full_state', False)
        self.declare_parameter('odometry', True)

        self.declare_parameter('camera',False)
        self.declare_parameter('camera_frame', "camera_frame")
        self.declare_parameter('camera_freq', 20)
        self.declare_parameter('pixel_width', 320)
        self.declare_parameter('pixel_height', 240)
        self.declare_parameter('camera_rgb_topic', "/camera/rgb")
        self.declare_parameter('camera_rgb_info_topic', "camera/rgb/camera_info")
        self.declare_parameter('camera_depth_image_topic', "/camera/depth/image_raw")
        self.declare_parameter('camera_depth_points_topic', "/camera/depth/points")
        self.declare_parameter('camera_depth_info_topic', "camera/depth/camera_info")
        self.declare_parameter('pointcloud_enabled', False)

        self.declare_parameter('lidar', False)
        self.declare_parameter('lidar_render', False)
        self.declare_parameter('lidar_angle_min', 2.2689)
        self.declare_parameter('lidar_angle_max', -2.2689)
        self.declare_parameter('lidar_freq', 20)
        self.declare_parameter('lidar_point_num', 360)
        self.declare_parameter('lidar_range_min', 0.25)
        self.declare_parameter('lidar_range_max', 5.0)
        self.declare_parameter('lidar_time_increment', 0.0)
        self.declare_parameter('lidar_topic', "/scan")
        self.declare_parameter('lidar_frame', "scan_frame")

        self.declare_parameter('lateral_friction', 1.0)
        self.declare_parameter('spinning_friction', 0.0065)

        self.declare_parameter('accurate_motor_model_enabled', False)
        self.declare_parameter('simple_motor_model_enabled', True)
        self.declare_parameter('torque_control_enabled', False)
        self.declare_parameter('external_disturbance', False)
        self.declare_parameter('external_disturbance_value', 2000)
        self.declare_parameter('external_disturbance_duration', 0.001)
        self.declare_parameter('external_disturbance_interval', 2)
        self.declare_parameter('ros_whole_body_state', True)
        self.declare_parameter('ros_whole_body_state_topic', "whole_body_state")
        self.declare_parameter('ros_imu_topic', "imu/data")
        self.declare_parameter('ros_joint_states_topic', "joint_states")

        self.declare_parameter('ros_robot_odom_topic', "robot_odom")
        self.declare_parameter('lcm_servo_cmd_channel', "SERVO_CMD")
        self.declare_parameter('lcm_servo_state_channel', "SERVO_STATE")
        self.declare_parameter('lcm_imu_channel', "IMU_DATA")
        self.declare_parameter('lcm_odom_channel', "ODOMETRY")
        self.declare_parameter('lcm_contact_sensor_channel', "CONTACT_SENSOR")
        self.declare_parameter('lcm_robot_state_channel', "ROBOT_STATE")
        self.declare_parameter('clock_topic', "clock")
        

        self.declare_parameter('ros_contact_flags_topic', "contact_flags")
        self.declare_parameter('joint_dir', [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        self.declare_parameter('joint_offset', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.declare_parameter('tau_max', [6.0, 6.0, 12.0])
        self.declare_parameter('gear_ratio', 10.0)
        self.declare_parameter('kt', 0.74)

        self.sim_freq = self.get_parameter('frequency').get_parameter_value().integer_value
        self.urdf_root = self.get_parameter("urdf_root").get_parameter_value().string_value
        self.world_name = self.get_parameter("world").get_parameter_value().string_value
        self.render = self.get_parameter("render").get_parameter_value().bool_value
        self.on_rack = self.get_parameter("on_rack").get_parameter_value().bool_value
        self.get_logger().info(f"On Rack: {self.on_rack}")
         
        self.camera_enabled = self.get_parameter("camera").get_parameter_value().bool_value
        if self.camera_enabled:
            self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
            self.camera_freq = self.get_parameter("camera_freq").get_parameter_value().integer_value
            self.pixel_width = self.get_parameter("pixel_width").get_parameter_value().integer_value
            self.pixel_height = self.get_parameter("pixel_height").get_parameter_value().integer_value
            self.camera_rgb_topic = self.get_parameter("camera_rgb_topic").get_parameter_value().string_value
            self.camera_rgb_info_topic = self.get_parameter("camera_rgb_info_topic").get_parameter_value().string_value
            self.camera_depth_image_topic = self.get_parameter("camera_depth_image_topic").get_parameter_value().string_value
            self.camera_depth_points_topic = self.get_parameter("camera_depth_points_topic").get_parameter_value().string_value
            self.camera_depth_info_topic = self.get_parameter("camera_depth_info_topic").get_parameter_value().string_value
            self.pointcloud_enabled = self.get_parameter("pointcloud_enabled").get_parameter_value().bool_value

        self.lidar_enabled = self.get_parameter("lidar").get_parameter_value().bool_value
        if self.lidar_enabled:
            self.lidar_render = self.get_parameter("lidar_render").get_parameter_value().bool_value
            self.lidar_angle_min = self.get_parameter("lidar_angle_min").get_parameter_value().double_value
            self.lidar_angle_max = self.get_parameter("lidar_angle_max").get_parameter_value().double_value
            self.lidar_freq = self.get_parameter("lidar_freq").get_parameter_value().integer_value
            self.lidar_point_num = self.get_parameter("lidar_point_num").get_parameter_value().integer_value
            self.lidar_range_min = self.get_parameter("lidar_range_min").get_parameter_value().double_value
            self.lidar_range_max = self.get_parameter("lidar_range_max").get_parameter_value().double_value
            self.lidar_time_increment = self.get_parameter("lidar_time_increment").get_parameter_value().double_value
            self.lidar_topic = self.get_parameter("lidar_topic").get_parameter_value().string_value
            self.lidar_frame = self.get_parameter("lidar_frame").get_parameter_value().string_value

        self.lateral_friction = self.get_parameter("lateral_friction").get_parameter_value().double_value
        self.spinning_friction = self.get_parameter("spinning_friction").get_parameter_value().double_value

        self.accurate_motor_model_enabled = self.get_parameter("accurate_motor_model_enabled").get_parameter_value().bool_value
        self.simple_motor_model_enabled = self.get_parameter("simple_motor_model_enabled").get_parameter_value().bool_value
        self.torque_control_enabled = self.get_parameter("torque_control_enabled").get_parameter_value().bool_value

        self.external_disturbance_enabled = self.get_parameter("external_disturbance").get_parameter_value().bool_value
        self.external_disturbance_value = self.get_parameter("external_disturbance_value").get_parameter_value().integer_value
        self.external_disturbance_duration = self.get_parameter("external_disturbance_duration").get_parameter_value().double_value
        self.external_disturbance_interval = self.get_parameter("external_disturbance_interval").get_parameter_value().integer_value

        self.ros_whole_body_state_enabled = self.get_parameter("ros_whole_body_state").get_parameter_value().bool_value
        self.ros_whole_body_state_topic = self.get_parameter("ros_whole_body_state_topic").get_parameter_value().string_value
        self.ros_imu_topic = self.get_parameter("ros_imu_topic").get_parameter_value().string_value

        self.ros_joint_states_topic = self.get_parameter("ros_joint_states_topic").get_parameter_value().string_value
        self.ros_robot_odom_topic = self.get_parameter("ros_robot_odom_topic").get_parameter_value().string_value
        self.lcm_servo_cmd_channel = self.get_parameter('lcm_servo_cmd_channel').get_parameter_value().string_value
        self.lcm_servo_state_channel = self.get_parameter('lcm_servo_state_channel').get_parameter_value().string_value
        self.lcm_imu_channel = self.get_parameter('lcm_imu_channel').get_parameter_value().string_value
        self.lcm_odom_channel = self.get_parameter('lcm_odom_channel').get_parameter_value().string_value
        self.lcm_contact_sensor_channel = self.get_parameter('lcm_contact_sensor_channel').get_parameter_value().string_value
        self.lcm_robot_state_channel = self.get_parameter('lcm_robot_state_channel').get_parameter_value().string_value
        self.clock_topic = self.get_parameter("clock_topic").get_parameter_value().string_value

        self.foot_contacts_enabled = self.get_parameter("foot_contacts").get_parameter_value().bool_value
        if self.foot_contacts_enabled:
            self.ros_contact_flags_topic = self.get_parameter("ros_contact_flags_topic").get_parameter_value().string_value

        self.full_state_enabled = self.get_parameter("full_state").get_parameter_value().bool_value
        self.lcm_odometry_enabled = self.get_parameter("odometry").get_parameter_value().bool_value

        self.joint_dir = self.get_parameter("joint_dir").get_parameter_value().integer_array_value
        self.joint_offset = self.get_parameter("joint_offset").get_parameter_value().integer_array_value

        self.tau_max = self.get_parameter("tau_max").get_parameter_value().double_array_value
        self.gear_ratio = self.get_parameter("gear_ratio").get_parameter_value().double_value
        self.kt = self.get_parameter("kt").get_parameter_value().double_value

        self.sim_period = 1.0/self.sim_freq

    def set_start_position(self):
        self.init_ref_pos[0] = -0.0
        self.init_ref_pos[1] = -0.4
        self.init_ref_pos[2] = 0.8
        self.init_ref_pos[3] = 0.0
        self.init_ref_pos[4] = 0.4
        self.init_ref_pos[5] = -0.8
        self.init_ref_pos[6] = 0.0
        self.init_ref_pos[7] = -0.4
        self.init_ref_pos[8] = 0.8
        self.init_ref_pos[9] = -0.0
        self.init_ref_pos[10] = 0.4
        self.init_ref_pos[11] = -0.8

    def init_simulation(self):
        self.env = MorsBulletEnv(
                urdf_root=self.urdf_root,
                sim_freq=self.sim_freq,
                world=self.world_name,
                hard_reset=False,
                render=self.render,
                on_rack=self.on_rack,
                self_collision_enabled=False,
                debug_mode=False,
                floating_camera=True,
                step_enabled=False,
                init_pos=[0, 0, .24],
                init_kin_scheme="x",

                accurate_motor_model_enabled=self.accurate_motor_model_enabled,
                simple_motor_model_enabled=self.simple_motor_model_enabled,
                torque_control_enabled=self.torque_control_enabled,
                motor_kp=0.0,
                motor_kd=0.0,
                motor_velocity_limit=np.inf,
                
                max_timesteps = np.inf,
                action_repeat=1,
                rew_scale = 1,
                distance_limit=float("inf"),
                observation_noise_stdev=0.0,
                normalization = False,

                camera_enabled=self.camera_enabled,
                lidar_enabled=self.lidar_enabled,
                ext_disturbance_enabled=self.external_disturbance_enabled,
                )

        self.env.set_friction(self.lateral_friction, self.spinning_friction)
        self.env.set_ext_forces_params(self.external_disturbance_value, 
                                       self.external_disturbance_duration*self.sim_freq, 
                                       self.external_disturbance_interval*self.sim_freq)
        # self.env.set_joint_max_velocity(40)

        if self.camera_enabled:
            self.env.camera.set_params(self.camera_freq, self.pixel_width, self.pixel_height, self.pointcloud_enabled)

        if self.lidar_enabled:
            self.env.lidar.set_params(render=self.lidar_render, 
                                      angle_min=self.lidar_angle_min, 
                                      angle_max=self.lidar_angle_max, 
                                      point_num=self.lidar_point_num, 
                                      range_min=self.lidar_range_min, 
                                      range_max=self.lidar_range_max)
            self.env.lidar.reset()

    def camera_loop(self):
        rate_1 = self.create_rate(self.camera_freq)
        it = 0
        while (1):
            self.env.camera.update()

            if self.env.camera.is_camera_updated():
                self.rgb_img = self.env.camera.get_rgb_image()
                self.depth_img = self.env.camera.get_depth_image()
                self.pointcloud_msg = self.env.camera.get_pointcloud()
                self.__pub_rgb_image(self.rgb_img)
                self.__pub_depth_image(self.depth_img)
                if self.pointcloud_enabled:
                    self.__pub_pointcloud(self.pointcloud_msg)

            rate_1.sleep()
        
    def loop(self):
        self.sim_it += 1

        self.env.set_kpkd(self.kp, self.kd)
        # print(self.kp)
        # self.env.set_kpkd(30.0, 0.1)
        # print(self.ref_pos, self.ref_vel, self.ref_torq)
        self.env.set_motor_commands(self.ref_pos, self.ref_vel, self.ref_torq)
        # print(f"ref pos: {self.ref_pos[8]:.2f} | kp: {self.kp[0]:.2f} | kd: {self.kd[0]:.2f}")

        observation, rew, done, _, _ = self.env.step(self.ref_data)

        self.contact_points, self.contact_flags = self.env.get_contact_flags()
        # print(self.contact_flags)
        self.motor_data["pos"], self.motor_data["vel"], self.motor_data["torq"], self.motor_data["name"] = self.env.get_motor_states()

        self.imu_data[7:10] = self.env.get_base_ang_vel()
        self.body_lin_pos, self.imu_data[3:7] = self.env.get_base_position_and_orientation()
        self.body_lin_vel = np.array(self.env.get_base_lin_vel(), float)
        self.imu_data[10:] = self.env.get_base_orientation_euler()
        self.imu_data[0:3] = (self.body_lin_vel - self.body_lin_vel_prev)/self.sim_period
        self.body_lin_vel_prev = self.body_lin_vel
        # print(f"{np.sqrt(self.body_lin_vel[0]**2 + self.body_lin_vel[1]**2):.2f}")

        theta = [0.0]*12
        for i in range(12):
            theta[i] = self.motor_data["pos"][i]
        r1_pos, l1_pos, r2_pos, l2_pos = self.fkine.solve(theta)
          
        self.__pub_clocks()
        self.__pub_imu_msg(self.imu_data)
        self.__pub_joint_states(self.motor_data)

        if self.foot_contacts_enabled:
            self.__pub_foot_contacts(self.contact_flags)
        
        if self.ros_whole_body_state_enabled:
            self.robot_tf.sendTransform(self.__fill_tf_message("map", "base_link", self.body_lin_pos, self.imu_data[3:7]))
            self.__pub_odom_msg(self.body_lin_pos, self.imu_data)
            self.__pub_robot_vel(self.body_lin_vel)
            self.__pub_whole_body_state(imu_data=self.imu_data,
                                        leg_data=self.motor_data,
                                        base_pos=self.body_lin_pos,
                                        contact_points=self.contact_points)
        
        if self.lcm_odometry_enabled:
            self.__pub_lcm_odom_msg(self.body_lin_pos, self.body_lin_vel, self.imu_data)

        if self.lidar_enabled and self.env.lidar.is_lidar_updated():
            self.__pub_lidar_message(self.env.lidar.get_data())

        if self.full_state_enabled:
            self.__pub_robot_state(imu_data=self.imu_data,
                                base_pos=self.body_lin_pos,
                                base_vel=self.body_lin_vel,
                                contact_states=self.contact_flags,
                                r1_pos=r1_pos,
                                l1_pos=l1_pos,
                                r2_pos=r2_pos,
                                l2_pos=l2_pos)

        self.map_state_msg(observation)
        self.lc_servo_state.publish(self.lcm_servo_state_channel, self.servo_state_msg.encode())

    def __pub_robot_vel(self, vel):
        self.robot_vel_msg.x = vel[0]
        self.robot_vel_msg.y = vel[1]
        self.robot_vel_msg.z = vel[2]
        self.vel_pub.publish(self.robot_vel_msg)

    def __pub_imu_msg(self, imu_data : list):
        self.lcm_imu_msg.linear_acceleration = imu_data[:3]
        self.lcm_imu_msg.angular_velocity = imu_data[7:10]
        self.lcm_imu_msg.orientation_euler = imu_data[10:]
        self.lcm_imu_msg.orientation_quaternion = imu_data[3:7]
        
        self.lc_imu.publish(self.lcm_imu_channel, self.lcm_imu_msg.encode())

        self.imu_msg.linear_acceleration.x = imu_data[0]
        self.imu_msg.linear_acceleration.y = imu_data[1]
        self.imu_msg.linear_acceleration.z = imu_data[2]
        self.imu_msg.angular_velocity.x = imu_data[7]
        self.imu_msg.angular_velocity.y = imu_data[8]
        self.imu_msg.angular_velocity.z = imu_data[9]
        self.imu_msg.orientation.x = imu_data[3]
        self.imu_msg.orientation.y = imu_data[4]
        self.imu_msg.orientation.z = imu_data[5]
        self.imu_msg.orientation.w = imu_data[6]
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = "base_link"
        self.imu_data_pub.publish(self.imu_msg)

    def __pub_joint_states(self, joint_states : dict):
        self.js_msg.name = []
        self.js_msg.position = []
        self.js_msg.velocity = []
        self.js_msg.effort = []
        # i = 0
        # for _ in joint_states["name"]:
        #     self.js_msg.name.append(joint_states["name"][i].decode('utf-8'))
        #     self.js_msg.position.append(joint_states["pos"][i]*self.leg_data_transform[i])
        #     self.js_msg.velocity.append(joint_states["vel"][i]*self.leg_data_transform[i])
        #     self.js_msg.effort.append(joint_states["torq"][i]*self.leg_data_transform[i])
        #     i += 1
        
        for o in self.leg_data_order:
            self.js_msg.name.append(joint_states["name"][o].decode('utf-8'))
            self.js_msg.position.append(joint_states["pos"][o]*self.leg_data_transform[o])
            self.js_msg.velocity.append(joint_states["vel"][o]*self.leg_data_transform[o])
            self.js_msg.effort.append(joint_states["torq"][o]*self.leg_data_transform[o])
            # i += 1

        self.js_msg.header.stamp = self.get_clock().now().to_msg()
        self.js_msg.header.frame_id = ""
        self.js_pub.publish(self.js_msg)

    def __pub_robot_state(self, imu_data, base_pos, base_vel, contact_states, r1_pos, l1_pos, r2_pos, l2_pos):
        for i in range(3):
            self.lcm_robot_state_msg.body.position[i] = base_pos[i]
            self.lcm_robot_state_msg.body.orientation[i] = imu_data[i+10]
            self.lcm_robot_state_msg.body.lin_vel[i] = base_vel[i]
            self.lcm_robot_state_msg.body.ang_vel[i] = imu_data[i+7]

            self.lcm_robot_state_msg.legs.r1_pos[i] = r1_pos[i]
            self.lcm_robot_state_msg.legs.l1_pos[i] = l1_pos[i]
            self.lcm_robot_state_msg.legs.r2_pos[i] = r2_pos[i]
            self.lcm_robot_state_msg.legs.l2_pos[i] = l2_pos[i]

        self.lcm_robot_state_msg.legs.contact_states = contact_states

        self.lc_robot_state.publish(self.lcm_robot_state_channel, self.lcm_robot_state_msg.encode())

    def __pub_whole_body_state(self, imu_data, leg_data, base_pos, contact_points):
        self.cur_time = self.get_clock().now().to_msg()
        self.wbs.header.stamp = self.cur_time
        self.wbs.header.frame_id = "map"
        self.wbs.time = float(self.cur_time.sec)
        # This represents the base state (CoM motion, angular motion and centroidal momenta)
        self.wbs.centroidal.com_position.x = base_pos[0]
        self.wbs.centroidal.com_position.y = base_pos[1]
        self.wbs.centroidal.com_position.z = base_pos[2]
        self.wbs.centroidal.base_orientation.x = imu_data[3]
        self.wbs.centroidal.base_orientation.y = imu_data[4]
        self.wbs.centroidal.base_orientation.z = imu_data[5]
        self.wbs.centroidal.base_orientation.w = imu_data[6]
        self.wbs.centroidal.base_angular_velocity.x = imu_data[7]
        self.wbs.centroidal.base_angular_velocity.y = imu_data[8]
        self.wbs.centroidal.base_angular_velocity.z = imu_data[9]
        # This represents the joint state (position, velocity, acceleration and effort)
        self.wbs.joints = []
        i = 0
        for _ in leg_data["name"]:
            js_msg = WBJointState()
            js_msg.name = leg_data["name"][i].decode('utf-8')
            js_msg.position = leg_data["pos"][i]
            js_msg.velocity = leg_data["vel"][i]
            self.wbs.joints.append(js_msg)
            i += 1
        # This represents the end-effector state (cartesian position and contact forces)
        self.wbs.contacts = []
        
        for contact_point in contact_points:
            contact_msg = WBContactState()
            contact_msg.name = "base_link"
            contact_msg.type = WBContactState.UNKNOWN
            contact_msg.pose.position.x = contact_point[5][0]
            contact_msg.pose.position.y = contact_point[5][1]
            contact_msg.pose.position.z = contact_point[5][2]
            contact_msg.wrench.force.z = contact_point[9]
            contact_msg.surface_normal.x = contact_point[7][0]
            contact_msg.surface_normal.y = contact_point[7][1]
            contact_msg.surface_normal.z = contact_point[7][2]
            contact_msg.friction_coefficient = 1.0
            self.wbs.contacts.append(contact_msg)
        self.wbc_state_pub.publish(self.wbs)

    def __fill_tf_message(self, parent_frame, child_frame, translation, rotation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = rotation[0]
        t.transform.rotation.y = rotation[1]
        t.transform.rotation.z = rotation[2]
        t.transform.rotation.w = rotation[3]
        return t
    
    def __pub_odom_msg(self, base_pos, imu_data):
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose.position.x = base_pos[0]
        self.odom.pose.pose.position.y = base_pos[1]
        self.odom.pose.pose.position.z = base_pos[2]
        self.odom.pose.pose.orientation.x = imu_data[3]
        self.odom.pose.pose.orientation.y = imu_data[4]
        self.odom.pose.pose.orientation.z = imu_data[5]
        self.odom.pose.pose.orientation.w = imu_data[6]

        self.odom_pub.publish(self.odom)

        t = self.__fill_tf_message(
            self.odom.header.frame_id, self.odom.child_frame_id, base_pos[0:3], imu_data[3:7])
        # because of SLAM
        self.robot_tf.sendTransform(t)

    def __pub_lcm_odom_msg(self, body_lin_pos, body_lin_vel, imu_data):
        self.lcm_odom_msg.position = body_lin_pos[:]
        self.lcm_odom_msg.orientation = imu_data[10:]
        self.lcm_odom_msg.lin_vel = body_lin_vel[:]
        self.lcm_odom_msg.ang_vel = imu_data[7:10]
        self.lc_odom.publish(self.lcm_odom_channel, self.lcm_odom_msg.encode())
    
    def __pub_rgb_image(self, img):
        self.img_rgb_msg.header.stamp = self.get_clock().now().to_msg()
        pil_img = pil.fromarray(img)
        self.img_rgb_msg.data = np.array(pil_img.convert('RGB')).tobytes()
        self.rgb_image_pub.publish(self.img_rgb_msg)
        self.rgb_info_pub.publish(self.cam_info_msg)

    def __pub_depth_image(self, img):
        self.img_depth_msg.header.stamp = self.get_clock().now().to_msg()
        self.img_depth_msg.data = np.array(img).tobytes()

        self.depth_image_pub.publish(self.img_depth_msg)
        self.depth_info_pub.publish(self.cam_info_msg)

    def __pub_pointcloud(self, pointcloud):
        self.depth_points_pub.publish(pointcloud)

    def __pub_foot_contacts(self, contact_flags):
        # self.contact_msg.header.stamp = self.get_clock().now().to_msg()
        self.contact_msg.contacts = contact_flags
        self.contact_pub.publish(self.contact_msg)

        self.lcm_contact_sensor_msg.contact_states = contact_flags[:]
        self.lc_contact.publish(self.lcm_contact_sensor_channel, self.lcm_contact_sensor_msg.encode())

    def __pub_lidar_message(self, hit_positions):
        self.lidar_msg.header.stamp = self.get_clock().now().to_msg()
        self.lidar_msg.ranges = hit_positions
        self.lidar_pub.publish(self.lidar_msg)

    def __pub_clocks(self):
        self.cur_time = self.get_clock().now().to_msg()
        self.delta_time_s = self.cur_time.sec - self.zero_time.sec
        self.clock_msg.clock.sec = self.delta_time_s
        self.clock_pub.publish(self.clock_msg)

    def get_cmd(self):
        # init LCM
        lc = lcm.LCM()
        subscription = lc.subscribe(self.lcm_servo_cmd_channel, self.cmd_handler)
        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass


    def cmd_handler(self, channel, data):
        msg = servo_cmd_msg.decode(data)
        # print(msg.kp)
        # print(f"Received message: {msg.position}")
        for i in range(12):
            self.ref_pos[i] = msg.position[i]
            self.ref_vel[i] = msg.velocity[i]
            self.ref_torq[i] = msg.torque[i] * self.kt / self.gear_ratio
            self.kp[i] = msg.kp[i]
            self.kd[i] = msg.kd[i]

    def map_state_msg(self, observation):
        for i in range(12):
            self.servo_state_msg.position[i] = self.joint_dir[i]*observation[i]-self.joint_offset[i]
            self.servo_state_msg.velocity[i] = self.joint_dir[0]*observation[i+12]
            self.servo_state_msg.torque[i] = self.joint_dir[0]*observation[i+24] * self.gear_ratio / self.kt

    def get_sim_period(self):
        return self.sim_period

def main(args=None):
    rclpy.init(args=args)

    hw_lvl_sim = Hardware_Level_Sim()

    rclpy.spin(hw_lvl_sim)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hw_lvl_sim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()