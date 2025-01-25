'''Plan'''
            try: 
                task = self.create_joint_task(leader_goal_joint, follower_goal_joint)
                if task.plan():
                    rospy.logwarn("Executing solution trajectory")
                    # task.publish(task.solutions[0])
                    
                    solution = task.solutions[0]
                    traj_collection = task.execute(solution)
                    
                    msg_to_mios.success = True
                    self.is_planning = False
                    
                    for traj_msg in traj_collection:
                        traj_list = []
                        solution_id = traj_msg.info.stage_id
                        joint_names = traj_msg.trajectory.joint_trajectory.joint_names
                        joint_trajectory = traj_msg.trajectory.joint_trajectory.points
                        print(str(solution_id) + ": ")
                        print(' '.join('%s' % x for x in joint_names))
                        if joint_names:
                            if "finger_joint" in joint_names[0]:
                                continue
                            else:
                                real_world_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'real_world_traj_task_'+str(solution_id)+'.txt')
                                # with open(real_world_file_path, 'w+') as f:
                                for point in joint_trajectory:
                                    traj_list.append(list(point.positions))
                                #         f.write(' '.join('%s' % x for x in point.positions))
                                #         # f.write(' ')
                                #         # f.write(' '.join('%s' % x for x in point.velocities))
                                #         f.write(' \n')
                                
                                # smoothing
                                traj = np.array(traj_list)
                                smooth_traj = rtb.tools.mstraj(traj, dt=0.001, tacc=0, qdmax=0.5)
                                
                                '''Send trajectories to robots'''
                                if not local_test:
                                    client_socket = socket.socket()  # instantiate
                                    
                                    # Skip finger joints and Send arm trajectories to robots
                                    if "panda_1" in joint_names[0]:
                                        response_robot_id = 1
                                        client_socket = leader_client
                                        server_host = self.leader_traj_host
                                        server_port = self.leader_traj_port
                                    elif "panda_2" in joint_names[0]:
                                        response_robot_id = 2
                                        client_socket = follower_client
                                        server_host = self.follower_traj_host
                                        server_port = self.follower_traj_port
                                    
                                    if msg_to_mios.robot_id:
                                        msg_to_mios.robot_id.append(response_robot_id)
                                    else:
                                        msg_to_mios.robot_id = [response_robot_id]
                                    if msg_to_mios.traj_id:
                                        msg_to_mios.traj_id.append(solution_id)
                                    else:
                                        msg_to_mios.traj_id = [solution_id]
                                
                                
                                    # client_socket.connect((server_host, server_port))
                                    # send the write or read command
                                    command = "write"
                                    client_socket.send(f"{command}".encode())
                                    # send the name of the file
                                    smooth_file_path = os.path.join(os.path.dirname(__file__), 'saved_trajectories', 'smooth_real_world_traj_'+str(solution_id)+'.txt')
                                    smooth_file_name = os.path.basename(smooth_file_path)
                                    rospy.loginfo("file name %s", smooth_file_name)
                                    client_socket.send(f"{os.path.basename(smooth_file_path)}".encode())
                                    rospy.loginfo("send file name to mios at %s", server_host)
                                    # send the trajectory
                                    joint_traj_data = pickle.dumps(smooth_traj.q)
                                    client_socket.sendall(joint_traj_data)
                                    rospy.loginfo("send smooth trajectory to mios at %s", server_host)
                                    # client_socket.close()
                    
            except Exception as ex:
                rospy.logerr("planning failed with exception\n%s%s", ex, task)
            
        # close sockets
        if not local_test:
            leader_client.close()
            follower_client.close()
            rospy.loginfo("close sockets")
        
        rospy.loginfo("response is %s", msg_to_mios)