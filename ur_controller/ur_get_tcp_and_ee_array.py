import time
import numpy as np
import cv2
import re
from UDPReceiver import	 UDPReceiver
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive



class GraspPoints:
    def __init__(self, 
				 file_to_read="intersections_data.txt"):
        self.file_to_read = file_to_read
        
        
    def read_last_line(self,filename):
        """Read the last line of a text file."""
        try:
            with open(filename, "r") as file:
                lines = file.readlines()
                if lines:
                    return lines[-1].strip()  # Return the last line without newline character
        except FileNotFoundError:
            print(f"File {filename} not found.")
            return None

    def get_tactile_sensor_intersection(self):
        # Receive a single set of intersection points
        intersection_points = self.read_last_line(self.file_to_read)
        # data = [int(value) for value in intersection_points.split(",")]
		
        # Extract digits using regex and convert them into integers
        data = [int(value) for value in re.findall(r'\d+', intersection_points)]


        if data:
            print(f"Intersectios: {data}")
            x1, y1, x2, y2 = list(data)

        if data is None or len(data) < 2:
            print("No intersection points received or insufficient data.")
            return None
        elif x1 == 0:
            return None
        return x1, y1, x2, y2
    
    def get_3d_intersections(self,ee_matrix):
        mmpp = 0.0634
        x1, y1, x2, y2 = self.get_tactile_sensor_intersection()

        if x1 == 0:
            print("No rope in sensor detected.")
            return None

        # Convert the points into numpy arrays
        a = np.array([-(y1-120) * mmpp / 1000, 0, (x1-160) * mmpp / 1000, 1]) 
        b = np.array([-(y2-120) * mmpp / 1000, 0, (x2-160) * mmpp / 1000, 1]) 
        print(a,b)
		# Transform the points into the world frame using the given matrix
        a_world = np.dot(ee_matrix, a)
        b_world = np.dot(ee_matrix, b)

        print('Intersections in world frame:', a_world[:3], b_world[:3])

        return a_world, b_world


if __name__ == "__main__":
    # Calculate the Rodrigues transform, converting the rotation vector to a rotation matrix (3x3)
    # rtde_frequency = 500.0

    # rtde_c = RTDEControl("192.168.1.2", rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
        # rtde_r = RTDEReceive("192.168.1.2")

    # ee_pose = rtde_r.getActualTCPPose()##get the tcp pose of the ur

    # print("EE Pose: ", ee_pose) # Print the TCP pose of the UR


    ee_pose = [-0.4529551466304076, 0.1568767089088942, 0.5521215593773965, 2.206288826528473, 2.109733985047995, 0.07793355578840384]

    x, y, z = ee_pose[:3]
    rx, ry, rz = ee_pose[3:]
    
    udp_host_1 = "10.157.174.101"
    udp_host_2 = "192.168.1.12"
    udp_port = 5060
    # receiver = UDPReceiver(udp_host_1, udp_port)
 
    rotation_vector = np.array([rx, ry, rz], dtype=np.float64)
    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

    ee_matrix = np.eye(4)  
    ee_matrix[:3, :3] = rotation_matrix
      
    ee_matrix[:3, 3] = [x, y, z]
    # print("EE Matrix: ", ee_matrix) # Print the EE matrix of the UR

    ee_array = ee_matrix.flatten(order='F') # 'F' stands for column-first storage


    tcp = ee_array[12:15]
    print("TCP Position: ", tcp) # Print the TCP position of the UR
    grasp_points = GraspPoints()
    a_world, b_world = grasp_points.get_3d_intersections(ee_matrix)
    print("two points in world frame", a_world[:3], b_world[:3])
    
    # receiver.send_data(
	#                     a_world[:3],        # intersection3d1
    #                     b_world[:3],        # intersection3d2
	#                     tcp.tolist(),       # tcp
	# 				    udp_host_2,         # Target IP
	# 					udp_port            # Target Port
	# 							)

