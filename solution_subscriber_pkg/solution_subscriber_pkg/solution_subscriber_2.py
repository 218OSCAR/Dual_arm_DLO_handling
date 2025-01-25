#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_task_constructor_msgs.msg import Solution  
import numpy as np
from roboticstoolbox import tools as rtb_tools
import socket
import os
import pickle
import time

class SolutionSubscriber(Node):
    def __init__(self):
        super().__init__('solution_subscriber')

        # Create a subscriber and subscribe to the /solution topic
        # self.solution_subscription = self.create_subscription(
        #     Solution,  # msg type
        #     '/solution',  # Topic name
        #     self.solution_callback,  
        #     10  # Queue size
        # )
        # self.solution_subscription  # Preventing garbage collection
        
            
    def solution_callback(self):
        """Callback function when receiving /solution message"""
        print(f"Received a solution message!")
        
        client_socket = socket.socket()  # instantiate
        server_host = '10.157.174.101'
        server_port = 12345
        client_socket.connect((server_host, server_port))
        print(f"Connect to robot")

        # start_joint_names = msg.start_scene.robot_state.joint_state.name
        # start_joint_positions = msg.start_scene.robot_state.joint_state.position

        # print the joint state
        # print("Start Joint names: %s" % ', '.join(start_joint_names))
        # print("Start Joint positions: %s" % ', '.join(map(str, start_joint_positions)))
        # Extract and print track points
        # for i, sub_traj in enumerate(msg.sub_trajectory):
        #     print(f"Sub-trajectory {i + 1}:")
        #     if not sub_traj.trajectory.joint_trajectory.joint_names:
        #         print("  Empty trajectory.")
        #         continue

        #     # Get the joint names and points in the trajectory
        #     solution_id = sub_traj.info.stage_id
        #     traj_joint_names = sub_traj.trajectory.joint_trajectory.joint_names
        #     traj_points = sub_traj.trajectory.joint_trajectory.points
            
            # # Extract joint trajectory points
            # print(f"  Trajectory Joint names: {', '.join(traj_joint_names)}")
            # traj_list = []
            # if traj_joint_names:
            # #     if "finger_joint" in traj_joint_names[0]:
            # #         continue
            # #     else:
            #         real_world_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'real_world_traj_task_'+str(solution_id)+'.txt')
            #         # print(f"File path: {real_world_file_path}")
            #         for point_idx, point in enumerate(traj_points):
            #             traj_list.append(list(point.positions))
            #         traj = np.array(traj_list)
            #         # Smoothing the trajectory
            #         smooth_traj = rtb_tools.mstraj(traj, dt=0.001, tacc=0.1, qdmax=0.5)
            #         print(f"Smooth trajectory: {smooth_traj.q}")
            #         # print(f"Smooth trajectory shape: {smooth_traj.q.shape}")
            #         # client_socket = socket.socket()  # instantiate
            #         # if "left" in joint_names[0]:  # left one is the ur robot
            #         #     robot_id = "Left UR"
            #         #     server_host = self.left_ur_host
            #         #     server_port = self.left_ur_port
            #         if "right" in traj_joint_names[0]:  # right one is the panda robot
            #             robot_id = "Right Panda"
            #         #     server_host = '10.157.174.101'
            #         #     server_port = 12345
            #         #     client_socket.connect((server_host, server_port))
        command = "write"
        client_socket.send(f"{command}".encode('utf-8'))
        # send the name of the file
        # smooth_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'smooth_real_world_traj_'+str(solution_id)+'.txt')
        # smooth_file_name = os.path.basename(smooth_file_path)
        smooth_file_path = 'smooth_real_world_traj_'+str(3)+'.txt'
        smooth_file_name = str(3)+'.txt'
        print(f"file name {smooth_file_name}")
        
        # client_socket.send(f"{os.path.basename(smooth_file_path)}".encode())
        # print(f"send fbasename(smooth_file_path)ile name to mios at {server_host}")
        client_socket.send(smooth_file_path.encode('utf-8'))  
        print(f"Sent file name: {smooth_file_path}")
        
        # send the trajectory
        # joint_traj_data = pickle.dumps(smooth_traj.q)
        # client_socket.sendall(joint_traj_data)
        # print(f"send smooth trajectory to mios at {server_host}")
                        # client_socket.close()

                    # if msg_to_mios.robot_id:
                    #     msg_to_mios.robot_id.append(robot_id)
                    # else:
                    #     msg_to_mios.robot_id = [robot_id]
                    # if msg_to_mios.traj_id:
                    #     msg_to_mios.traj_id.append(solution_id)
                    # else:
                    #     msg_to_mios.traj_id = [solution_id]
                    # client_socket.connect((server_host, server_port))
                    # send the write or read command
                    



            # print(f"  Joint names: {', '.join(traj_joint_names)}")
            # for point_idx, point in enumerate(traj_points):
            #     positions = list(point.positions)
            #     velocities = list(point.velocities)
            #     print(f"    Point {point_idx + 1}: Positions = {positions}, Velocities = {velocities}")



def main(args=None):
    print(f"Received a solution message!")
        
    client_socket = socket.socket()  # instantiate
    server_host = '10.157.174.101'
    
    server_port = 12345
    client_socket.connect((server_host, server_port))
    print(f"Connect to robot")
    
    time.sleep(2)
    
    
    command = "write"
    client_socket.send(f"{command}".encode('utf-8'))
    # send the name of the file
    # smooth_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'smooth_real_world_traj_'+str(solution_id)+'.txt')
    # smooth_file_name = os.path.basename(smooths_file_path)
    smooth_file_path = 'smooth_real_world_traj_'+str(3)+'.txt'
    smooth_file_name = str(3)+'.txt'
    print(f"file name {smooth_file_name}")
    time.sleep(2)
    # client_socket.send(f"{os.path.basename(smooth_file_path)}".encode())
    # print(f"send fbasename(smooth_file_path)ile name to mios at {server_host}")
    client_socket.send(smooth_file_path.encode('utf-8'))  
    print(f"Sent file name: {smooth_file_path}")


if __name__ == '__main__':
    main()
