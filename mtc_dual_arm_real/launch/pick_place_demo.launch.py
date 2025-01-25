from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
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
        ],
    )

    return LaunchDescription([pick_place_demo])