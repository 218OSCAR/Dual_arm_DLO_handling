"""from rtde_control import RTDEControlInterface as RTDEControl



rtde_frequency = 500.0

rtde_c = RTDEControl("192.168.1.2", rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)

rtde_c.moveL([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04], 0.5, 0.3)"""



## https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#servoj-example

import socket
import time

from rtde_control import RTDEControlInterface as RTDEControl
from robotiq_gripper_control import RobotiqGripper
import socket
import json


def udp_send(message, target_ip, target_port):
    """Send a UDP message."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (target_ip, target_port))
    sock.close()

def udp_receive(local_ip, local_port, buffer_size=1024):
    """Receive a UDP message."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((local_ip, local_port))
    print(f"Listening for UDP messages on {local_ip}:{local_port}")
    data, addr = sock.recvfrom(buffer_size)
    sock.close()
    return data.decode()


if __name__ == "__main__":
    # Initialize RTDE Control Interface

    rtde_frequency = 500.0
    local_ip = "192.168.1.7"
    panda_ip = "192.168.1.7"#the ip of the computer to control the panda
    udp_port = 5005
    rtde_c = RTDEControl("192.168.1.2", rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
    gripper = RobotiqGripper(rtde_c)

    # Function to read trajectory data from a file

    def read_trajectory(file_path):
        trajectory = []
        with open(file_path, 'r') as file:
            for line in file:
                joint_values = list(map(float, line.strip().split()))
        # Add velocity, acceleration, and blend values to each joint pose
        #velocity = 0.5
        #acceleration = 0.5
        #blend = 0.02 # Adjust blend value as needed
                joint_pose = joint_values #+ [velocity, acceleration]
                trajectory.append(joint_pose)
        return trajectory

    # Define file paths
    file_paths = ["./src/ur_controller/smooth_real_world_traj_4.txt", "./src/ur_controller/smooth_real_world_traj_6.txt", "./src/ur_controller/smooth_real_world_traj_12.txt", "./src/ur_controller/smooth_real_world_traj_13.txt"]
    # file_paths = [ "./src/ur_controller/smooth_real_world_traj_6.txt", "./src/ur_controller/smooth_real_world_traj_12.txt", "./src/ur_controller/smooth_real_world_traj_13.txt"]

    # Read all trajectories and combine them
    full_trajectory = []

    for file_path in file_paths:
        full_trajectory.extend(read_trajectory(file_path))

        time.sleep(0.1)

    # Execute the combined trajectory

    velocity = 0.5

    acceleration = 0.5

    dt = 1.0/500 # 2ms

    lookahead_time = 0.1

    gain = 300
    init_q = [2.3000001337865097, -1.5599999372122937, -1.4700001079028475, -1.7000002280364026, 1.57999970794,0.7200001496299335]

    rtde_c.moveJ(init_q)


    for i in range(len(full_trajectory)):

        t_start = rtde_c.initPeriod()

        rtde_c.servoJ(full_trajectory[i], velocity, acceleration, dt, lookahead_time, gain)

        rtde_c.waitPeriod(t_start)

    # rtde_c.GetActualTCPPose()##get the tcp pose of the ur

    rtde_c.servoStop()

    rtde_c.stopScript()
    
    
    
    # # UR listens and opens gripper
    # print("UR: Waiting for Panda grasp completion message...")
    # msg = udp_receive(local_ip, udp_port)
    # if msg == "grasp_done":
    #     print("UR: Received grasp completion message, opening gripper...")
    #     gripper.activate()  # returns to previous position after activation
    #     gripper.set_force(50)  # from 0 to 100 %
    #     gripper.set_speed(100)  # from 0 to 100 %
    #     gripper.open()
    #     udp_send("gripper_opened", panda_ip, udp_port)
    # else:
    #     print("UR: Unexpected message received.")
    # # Activate the gripper and initialize force and speed
    

    """rtde_c.servoJ(full_trajectory)

    rtde_c.stopScript()"""