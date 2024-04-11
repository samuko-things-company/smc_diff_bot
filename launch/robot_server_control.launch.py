import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node


def generate_launch_description():
    # delare any path variable
    my_pkg_path = get_package_share_directory('smc_test_bot')  
  
    # create needed nodes or launch files
    rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(my_pkg_path,'launch','rsp.launch.py')]
            ), 
            launch_arguments={'use_sim_time': 'false'}.items()
    )

    robot_controllers = os.path.join(my_pkg_path,'config','diff_drive_controller.yaml')

    # see -> https://github.com/ros-controls/ros2_control_demos/blob/humble/example_2/bringup/launch/diffbot.launch.py
    # see -> https://control.ros.org/master/doc/ros2_control/controller_manager/doc/userdoc.html
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/diff_drive_controller/odom", "/odom"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )    


    # Delay controller manager for some time for full load up
    delay_controller_manager_for_some_time = TimerAction(period=5.0, actions=[controller_manager])

    # Delay start of joint_state_broadcaster until controller_manager starts
    delay_joint_state_broadcaster_spawner_until_controller_manager_starts = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    start_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )


     # Create the launch description and populate
    ld = LaunchDescription()
    

    # Add the nodes to the launch description
    ld.add_action(rsp)

    # ld.add_action(delay_controller_manager_for_some_time)
    # ld.add_action(delay_joint_state_broadcaster_spawner_until_controller_manager_starts)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(start_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner)

    
    return ld      # return (i.e send) the launch description for excecution

