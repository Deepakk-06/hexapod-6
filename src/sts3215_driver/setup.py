from setuptools import setup, find_packages

package_name = 'sts3215_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Low level serial driver for STS3215 bus servos',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = sts3215_driver.driver_node:main',
        ],
    },
)
