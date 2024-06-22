import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    pkg_name = 'robot_control'
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds/pal_office.world')
    world_path = os.path.join(get_package_share_directory(pkg_name), 'worlds')

    gz_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=[
                                               EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                                                    default_value=''),
                                                                    world_path])

    #Incluyo el anterior launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory(pkg_name), 'launch/rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    #Incluyo gazebo  on el mundo

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), 
        launch_arguments=[('gz_args', [f' -r {world_file}'])])

    joint_state_publisher_node =  Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            parameters=[{'use_sim_time' : True}])

    #Rviz 2 configuration
    """
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(get_package_share_directory(pkg_name), )
        ]
    )
    """

    #Spawneo al robot

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_robot',
                                   '-x', '0.0',
                                   '-y', '0.0',],
                        output = 'screen')
    
    """
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )
    """
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_broad'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_cont'],
        output='screen'
    )
    
    #gz ros Bridge

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',                   
                   ],
        parameters=[{'qos_overrides./model/my_robot.subscriber.reliability': 'reliable'}],
        output='screen'
    )
    
    return LaunchDescription([
        gz_resource_path,
        bridge,
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_controller],
            )
        ),
        rsp,
        joint_state_publisher_node, 
        #diff_drive_spawner,
        #joint_broad_spawner,      
        spawn_entity,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        
    ])
