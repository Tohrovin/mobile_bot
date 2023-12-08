# import os

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, TimerAction
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessStart

# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch.actions import IncludeLaunchDescription


# def generate_launch_description():


#     # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
#     # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

#     package_name='mobile_bot' 
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     use_ros2_control = LaunchConfiguration('use_ros2_control')

#     robot_description_topic = '/robot_description'

#     rsp = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','rsp.launch.py'
#                 )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
#     )

#     joystick = IncludeLaunchDescription(
#                 PythonLaunchDescriptionSource([os.path.join(
#                     get_package_share_directory(package_name),'launch','joystick.launch.py'
#                 )])
#     )


#     twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
#     twist_mux = Node(
#             package="twist_mux",
#             executable="twist_mux",
#             parameters=[twist_mux_params],
#             remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
#         )

    


#     robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

#     controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

#     # controller_manager = Node(
#     #     package="controller_manager",
#     #     executable="ros2_control_node",
#     #     parameters=[{'robot_description': robot_description},
#     #                 controller_params_file]
#     # )
    
#     controller_manager = Node(
#         package="controller_manager",
#         executable="ros2_control_node",
#         parameters=[{'robot_description': robot_description_topic}, controller_params_file]
#     )

#     delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

#     diff_drive_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["diff_cont"],
#     )

#     delayed_diff_drive_spawner = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=controller_manager,
#             on_start=[diff_drive_spawner],
#         )
#     )

#     joint_broad_spawner = Node(
#         package="controller_manager",
#         executable="spawner",
#         arguments=["joint_broad"],
#     )

#     delayed_joint_broad_spawner = RegisterEventHandler(
#         event_handler=OnProcessStart(
#             target_action=controller_manager,
#             on_start=[joint_broad_spawner],
#         )
#     )


#     # Code for delaying a node (I haven't tested how effective it is)
#     # 
#     # First add the below lines to imports
#     # from launch.actions import RegisterEventHandler
#     # from launch.event_handlers import OnProcessExit
#     #
#     # Then add the following below the current diff_drive_spawner
#     # delayed_diff_drive_spawner = RegisterEventHandler(
#     #     event_handler=OnProcessExit(
#     #         target_action=spawn_entity,
#     #         on_exit=[diff_drive_spawner],
#     #     )
#     # )
#     #
#     # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner



#     # Launch them all!
#     return LaunchDescription([
#         rsp,
#         joystick,
#         twist_mux,
#         delayed_controller_manager,
#         delayed_diff_drive_spawner,
#         delayed_joint_broad_spawner
#     ])


from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mobile_bot"), "description", "robot.urdf.xacro"]
            ),
        ]
    )
    package_name = "mobile_bot"
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(f"{package_name}"),
            "config",
            "my_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_cont/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    nodes = [
        joystick,
        twist_mux,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
