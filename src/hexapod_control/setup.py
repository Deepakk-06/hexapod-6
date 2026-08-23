from setuptools import setup, find_packages

package_name = 'hexapod_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/servo_config.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml', 'pyserial'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Gait engine, calibration, teleop, bring-up helpers',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_node = hexapod_control.gait_node:main',
            'stand_up = hexapod_control.stand_up:main',
            'keyboard_teleop = hexapod_control.keyboard_teleop:main',
            'servo_check = hexapod_control.servo_check:main',
            'joint_jog = hexapod_control.joint_jog:main',
            'capture_pose = hexapod_control.capture_pose:main',
            'odometry_node = hexapod_control.odometry_node:main',
            'leg_angle_monitor = hexapod_control.leg_angle_monitor:main',
        ],
    },
)
