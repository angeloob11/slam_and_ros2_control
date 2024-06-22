import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    
    #Revisa si podemos usar el tiempo de simulación
    use_sim_time = LaunchConfiguration('use_sim_time')

    #leer los archivos urdf
    pkg_path = os.path.join(get_package_share_directory('robot_control'))
    xacro_file = os.path.join(pkg_path, 'description/robot.xacro.urdf')
    robot_description_config = xacro.process_file(xacro_file)

    #Crea el Robot State Publisher
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    #Crea la launch description

    return LaunchDescription([
        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
        ),
        node_robot_state_publisher,
    ])
