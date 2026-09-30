from setuptools import setup


package_name = 'mors_experiments_sim'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='vldanilov90@gmail.com',
    description='Scripted simulation experiments for Mors robot locomotion.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exp1 = mors_experiments_sim.exp1_max_t_sw:main',
            'exp2a = mors_experiments_sim.exp2a_curvilinear:main',
            'exp2b = mors_experiments_sim.exp2b_curvilinear:main',
            'exp2c = mors_experiments_sim.exp2c_curvilinear:main',
            'exp2d = mors_experiments_sim.exp2d_linear:main',
        ],
    },
)
