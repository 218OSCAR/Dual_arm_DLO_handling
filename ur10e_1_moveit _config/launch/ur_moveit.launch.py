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
    description_package = "ur10e_1_moveit_config"
    description_file = "ur.urdf.xacro"
    moveit_config_package = "ur10e_1_moveit_config"
    moveit_joint_limits_file = "joint_limits.yaml"
    moveit_config_file = "ur.srdf"
    use_sim_time = False
    launch_rviz = "true"
    # ##如果调用MoveItConfigsBuilder函数，则连不上ur的启动的controller的launch文件，需要把controller.launch写入demo.launch中
    # moveit_config = (
    #     MoveItConfigsBuilder("ur10e_1")
    #     .robot_description(file_path="config/ur.urdf.xacro",
    #                     #    mappings={
    #                     #         "ros2_control_hardware_type": LaunchConfiguration(
    #                     #             "ros2_control_hardware_type"
    #                     #         )
    #                     #     },
    #                        )
    #     .robot_description_semantic(file_path="srdf/ur.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .planning_pipelines(pipelines=["ompl"])
    #     .to_moveit_configs()
    # )
    # # print(moveit_config.to_dict())
    # # Start the actual move_group node/action server
    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[moveit_config.to_dict(),{"use_sim_time": use_sim_time}],
    #     arguments=["--ros-args", "--log-level", "info"],
    # )
    ##相当于重新实现了MoveItConfigsBuilder的功能，MoveItConfigsBuilder 处理 xacro 文件并最终生成一个 展开宏后的URDF内容
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "config", description_file]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    #和MoveitConfigBuilder函数里的joint_limits对应
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package,
            os.path.join("config", moveit_joint_limits_file),
        )
    }
    
    ##和MoveitConfigBuilder函数里的palnning_pipelines对应
    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    ##和MoveitConfigBuilder函数里的trajectory_execution对应
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # the scaled_joint_trajectory_controller does not work on fake hardware
    change_controllers = "true"
    if change_controllers == "true":
        controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
        controllers_yaml["joint_trajectory_controller"]["default"] = True

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        # Execution time monitoring can be incompatible with the scaled JTC
        "trajectory_execution.execution_duration_monitoring": False,
    }
    ##和MoveitConfigBuilder函数里的planning_scene_monitor对应
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    # # rviz with moveit configuration
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    # )
    rviz_config = os.path.join(
        get_package_share_directory("ur10e_1_moveit_config"),
        "rviz/view_robot.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,

            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.planning_pipelines,
            # moveit_config.robot_description_kinematics,
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )
        

    return LaunchDescription(
       [
            rviz_node,
            move_group_node,
       ]
    )
