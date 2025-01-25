from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Declare the scene file argument
    scene_file_arg = DeclareLaunchArgument(
        "scene_file",
        default_value="/home/tp2/ws_humble/scene/mongodb_8.scene",  # Default file path
        description="Path to the .scene file to be loaded",
    )  
    
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda_ur")
        .robot_description(file_path="config/panda_ur.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )


    # MTC Demo node
    load_scene= Node(
        package="mtc_dual_arm_panda_ur",
        executable="load_scene",
        # package="moveit2_tutorials",
        # executable="mtc_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
        arguments=[LaunchConfiguration("scene_file")],
    )
    return LaunchDescription([scene_file_arg, load_scene])