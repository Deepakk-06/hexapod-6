from setuptools import setup, find_packages

package_name = 'mpu6050_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='MPU6050 IMU driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = mpu6050_driver.imu_node:main',
        ],
    },
)
