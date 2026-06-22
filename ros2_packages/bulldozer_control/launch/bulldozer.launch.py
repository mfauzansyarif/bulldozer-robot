import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import (get_package_share_directory)

def generate_launch_description():
    #RVIZ2
    rviz_config = os.path.join(
        get_package_share_directory(
            'bulldozer_control'
        ),
        'config',
        'bulldozer.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            rviz_config
        ],
        output='screen'
    )
    #GUI
    gui_node = Node(
        package='bulldozer_gui',
        executable='control_panel',
        name='control_panel',
        output='screen'
    )
    return LaunchDescription([
        #JOYSTICK
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        #CONTROL
        Node(
            package='bulldozer_control',
            executable='teleop_node',
            name='teleop_node',
            output='screen'
        ),
        #SERIAL
        Node(
            package='bulldozer_control',
            executable='serial_node',
            name='serial_node',
            output='screen'
        ),
        #AUTO
        Node(
            package='bulldozer_control',
            executable='auto_node',
            name='auto_node',
            output='screen'
        ),
        #SELECTOR
        Node(
            package='bulldozer_control',
            executable='selector_node',
            name='selector_node',
            output='screen'
        ),
        #ODOM
        Node(
            package='bulldozer_control',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),
        rviz_node,
        gui_node,
    ])