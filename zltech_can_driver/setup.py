from glob import glob

from setuptools import setup

package_name = 'zltech_can_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='msvn robotics',
    maintainer_email='msvnroboticsteam@gmail.com',
    description='Driver CAN 2 banh vi sai (ODrive Can-sync, Waveshare USB-CAN-A)',
    license='TODO',
    entry_points={
        'console_scripts': [
            'can_bridge = zltech_can_driver.can_bridge:main',
            'teleop = zltech_can_driver.teleop:main',
            'calib = zltech_can_driver.calib:main',
            'clear_errors = zltech_can_driver.clear_errors:main',
            'drive_test = zltech_can_driver.drive_test:main',
            'error_monitor = zltech_can_driver.error_monitor:main',
        ],
    },
)
