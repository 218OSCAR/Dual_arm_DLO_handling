# Introduction
 - Dual arm (Franka panda robot and UR robot) DLO handling task in the simulation environment and in the real world.
 - sensor1 - camera (realsense D435i) captures both RGB frames and depth maps to generate an initial 3D model of the deformable linear object.
 - sensor2 - tactile sensor (gelsight mini) provides localized, contact-based measurements that enhance the accuracy and reliability of the reconstructed 3D representation
 
# Installation
## Ubuntu
- 22.04
  
## ROS2
- install  **[ros-humble](https://docs.ros.org/en/humble/Installation.html)** into your Ubuntu
 
## MoveIt2
- install **[moveit-humble](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)** into your moveit workspace.

## MTC
- install **[moveit_task_constructor](https://github.com/218OSCAR/moveit_task_constructor)** into your moveit workspace to replace the original one.

## UR robot required packages
- install **[Universal_Robots_Client_Library](https://github.com/218OSCAR/Universal_Robots_Client_Library)** into your moveit workspace.
- install **[Universal_Robots_ROS2_Description](https://github.com/218OSCAR/Universal_Robots_ROS2_Description)** into your moveit workspace.
- install **[Universal_Robots_ROS2_Driver](https://github.com/218OSCAR/Universal_Robots_ROS2_Driver)** into your moveit workspace.

## Mios for controlling the panda robot in the real world
- install **[mios-wiring](https://github.com/218OSCAR/mios-wiring)** into the home folder.


# Description of the packages
 - **ur10e_1_moveit _config**: Change the moveit_config of the UR robot to the same type of panda robot, amd add a gripper(robotiq_2f85).
 - **dual_arm_panda_ur_moveit_config**: Implement that ur10e and panda can be loaded with a demo.launch.py file, and both can be planned in the same simulation environment.
   ![dual_arm_moveit_config](https://github.com/user-attachments/assets/fe7457d7-98ab-4331-9281-ad3a4020113b)
 - **mtc_dual_arm_panda_ur**: The MTC task for dual arm(panda and UR10e) to handle an object in the simulation environment.
 - **mtc_dual_arm_real**:  The MTC task for dual arm(panda and UR10e) to handle the DLO in the real world environment.
 - **solution_subscriber_pkg**: Listen to the trajectory planned by the MTC and send it to the traj_server of panda and UR10e(in the real world environment).


# Simulation
 - cd your moveit workspace
```bash
source install/setup.bash
```
  
- run `mtc_demo.launch.py` in `mtc_dual_arm_panda_ur` to launch the dual arm in the rviz.
```bash
ros2 launch mtc_dual_arm_panda_ur mtc_demo.launch.py
```

- After launch the mtc_demo.launch.py, you should first add the planning scene iwb_lab_workspace_4.scene

- run `pick_place_demo.launch.py ` in `mtc_dual_arm_panda_ur` to start the handling task.
```bash
ros2 launch mtc_dual_arm_panda_ur pick_place_demo.launch.py 
```









