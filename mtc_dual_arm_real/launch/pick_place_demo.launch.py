from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    skip_handover_flag = DeclareLaunchArgument(
        'skip_handover',
        default_value='false',
        description='Flag to skip hand-over operation or not'
    )
    
    # moveit_config = MoveItConfigsBuilder("dual_arm_panda_ur").to_dict()
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda_ur")
        .robot_description(file_path="config/panda_ur.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    ).to_dict()
    
    # MTC Demo node
    pick_place_demo = Node(
        package="mtc_dual_arm_real",
        executable="mtc_dual_arm_node",
        output="screen",
        parameters=[
            moveit_config,
            {"skip_handover": LaunchConfiguration('skip_handover')}
        ]
    )

    return LaunchDescription([skip_handover_flag, pick_place_demo])