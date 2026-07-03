from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mors_experiments'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name+'/config/', glob('config/*')),
        ('share/' + package_name+'/srv/', glob('srv/*')),
        ('share/' + package_name+'/msg/', glob('msg/*')),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='vldanilov90@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exp1 = mors_experiments.exp1_trot_flat:main',
            'exp2 = mors_experiments.exp2_gait_switch:main',
            'exp3 = mors_experiments.exp3_ext_force:main',
            'exp4 = mors_experiments.exp4_uneven_terrain:main',
            'exp5 = mors_experiments.exp5_kmu:main',
            'exp6 = mors_experiments.exp6_wbic_body_ctrl:main',
        ],
    },
)
