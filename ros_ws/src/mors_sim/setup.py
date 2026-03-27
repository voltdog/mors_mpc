from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'mors_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name+'/config/', glob('config/*')),
        ('lib/' + package_name+'/urdf/', glob('urdf/*')),
        ('lib/' + package_name+'/rviz/', glob('rviz/*')),
        ('lib/' + package_name+'/meshes/', glob('meshes/*')),
        # ('share/' + package_name+'/models/', glob('models/*')),
        ('lib/' + package_name+'/worlds/', glob('worlds/*')),
        ('lib/' + package_name + '/additional', [package_name+'/additional/mors_gym_env.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/forward_kinematics.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/motor_simple.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/motor_accurate.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/world_creator.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/lidar.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/camera.py']),
        ('lib/' + package_name + '/additional', [package_name+'/additional/gazebo_world_parser.py']),

        # ('lib/' + package_name + '/additional', [package_name+'/additional_mujoco/mors_gym_env.py']),
        # ('lib/' + package_name, [package_name+'/lcm_msgs/servo_cmd_msg.py']),
        # ('lib/' + package_name, [package_name+'/lcm_msgs/servo_state_msg.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoggi',
    maintainer_email='vldanilov90@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'mors_sim = mors_sim.mors_sim:main',
        ],
    },
)
