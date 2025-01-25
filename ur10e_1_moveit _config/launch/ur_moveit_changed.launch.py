import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    OrSubstitution,
)
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
     # Initialize Arguments
    ##如果调用MoveItConfigsBuilder函数，则连不上ur的启动的controller的launch文件，需要把controller.launch写入demo.launch中
    moveit_config = (
        MoveItConfigsBuilder("ur10e_1")
        .robot_description(file_path="config/ur.urdf.xacro",
                           )
        .robot_description_semantic(file_path="srdf/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    # print(moveit_config.to_dict())
    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )
   
    rviz_config = os.path.join(
        get_package_share_directory("ur10e_1_moveit_config"),
        "rviz/view_robot.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            # {
            #     "use_sim_time": use_sim_time,
            # }
        ],
    )
        
    
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur10e_1_moveit_config"),
        "config/",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "ur_manipulator_controller",
        "gripper_controller"
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
       [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
       ]
        + load_controllers
    )
