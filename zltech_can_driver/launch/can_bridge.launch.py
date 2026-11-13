"""
Launch can_bridge (2 banh ODrive Can-sync qua Waveshare USB-CAN-A).
  ros2 launch zltech_can_driver can_bridge.launch.py
  ros2 launch zltech_can_driver can_bridge.launch.py invert:=TRAI port:=/dev/ttyUSB0
  ros2 launch zltech_can_driver can_bridge.launch.py publish_tf:=true cmd_vel_topic:=/cmd_vel_nav
Lai bang ban phim (terminal khac):  ros2 run zltech_can_driver teleop
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params = os.path.join(
        get_package_share_directory('zltech_can_driver'), 'config', 'can_bridge_params.yaml')

    # Chi ghi de tham so khi launch argument khac rong -> mac dinh lay tu yaml
    overrides = {}
    for name in ('invert', 'port', 'cmd_vel_topic', 'odom_topic', 'base_frame'):
        val = LaunchConfiguration(name).perform(context)
        if val != '__yaml__':
            overrides[name] = val
    publish_tf = LaunchConfiguration('publish_tf').perform(context)
    if publish_tf != '__yaml__':
        overrides['publish_tf'] = publish_tf.lower() == 'true'

    node = Node(
        package='zltech_can_driver',
        executable='can_bridge',
        name='can_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[params, overrides],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('invert', default_value='__yaml__',
                              description='banh lap nguoc: "" | TRAI | PHAI (mac dinh theo yaml)'),
        DeclareLaunchArgument('port', default_value='__yaml__',
                              description='cong adapter, "" = tu do (mac dinh theo yaml)'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='__yaml__',
                              description='topic cmd_vel dau vao (mac dinh theo yaml)'),
        DeclareLaunchArgument('odom_topic', default_value='__yaml__',
                              description='topic odom (navigation dung /odom, mac dinh theo yaml)'),
        DeclareLaunchArgument('base_frame', default_value='__yaml__',
                              description='frame base (mac dinh theo yaml)'),
        DeclareLaunchArgument('publish_tf', default_value='__yaml__',
                              description='true | false: phat TF odom -> base_link (mac dinh theo yaml)'),
        OpaqueFunction(function=launch_setup),
    ])
