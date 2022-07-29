from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vgc10_control',
            executable='OnRobotVGStatusListener',
            output="screen",
            name='OnRobotVGStatusListener',
            parameters=[
                  {'/onrobot/ip': '192.168.1.1'},
                  {'/onrobot/port': '502'},
                  {'dummy': 'false'},
            ]
        ),
        Node(
            package='vgc10_control',
            executable='OnRobotVGTcpNode',
            output="screen",
            name='OnRobotVGTcpNode',
            parameters=[
                  {'/onrobot/ip': '192.168.1.1'},
                  {'/onrobot/port': '502'},
                  {'dummy': 'false'},
            ]
        ),
    ])