import socket
'''
reference: https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/ by Adrian Rosebrock 
'''

import numpy as np
import os
import sys
# import pykinect_azure as pykinect
import tqdm
import threading
import copy
import pickle
import argparse
from data_processing.utils.datasets_utils import Dataloader

# device's IP address
# SERVER_HOST = socket.gethostname()
SERVER_HOST = "10.157.174.101"
SERVER_PORT = 12345 
# receive 4096 bytes each time
BUFFER_SIZE = 4096
SEPARATOR = "<SEPARATOR>"


parser = argparse.ArgumentParser(description='For application on different PCs.')

parser.add_argument('--server_host', type=str, default="10.157.174.101",
					help='ip address of the target robot pc')
parser.add_argument('--server_port', type=int, default=12345,
					help='port of the target robot pc to run the server')
parser.add_argument('--buffer_size', type=int, default=4096,
					help='buffer size of the socket client')

args = parser.parse_args()


def server_program(host, port):
	# server_program(local_ip, port)	
	server_socket = socket.socket()  # get instance
	# look closely. The bind() function takes tuple as argument
	server_socket.bind((host, port))  # bind host address and port together

	# configure how many client the server can listen simultaneously
	server_socket.listen(2)
	client_socket, address = server_socket.accept()  # accept new connection
	print("Connection from: " + str(address))
 
	# receive using client socket, not server socket
	received = client_socket.recv(BUFFER_SIZE).decode()
	filename, filesize = received.split(SEPARATOR)
	# remove absolute path if there is
	filename = "received" + os.path.basename(filename)
	# convert to integer
	filesize = int(filesize)
	
	# start receiving the file from the socket
	# and writing to the file stream
	progress = tqdm.tqdm(range(filesize), f"Receiving {filename}", unit="B", unit_scale=True, unit_divisor=1024)
	with open(filename, "wb") as f:
		while True:
			# read 1024 bytes from the socket (receive)
			bytes_read = client_socket.recv(BUFFER_SIZE)
			if not bytes_read:    
				# nothing is received
				# file transmitting is done
				break
			# write to the file the bytes we just received
			f.write(bytes_read)
			# update the progress bar
			progress.update(len(bytes_read))

	# close the client socket
	client_socket.close()


if __name__ == '__main__':
	# get the hostname
	local_ip = socket.gethostname()
	port = 12345  # initiate port no above 1024
 
	# server_program(local_ip, port)	
	server_socket = socket.socket()  # get instance
	# look closely. The bind() function takes tuple as argument
	server_socket.bind((args.server_host, args.server_port))  # bind host address and port together
	print("server started at " ,args.server_host)
 
	sources = {"ForceControl":["f_ext", "dx", "x", "ff"]}

	# configure how many client the server can listen simultaneously
	server_socket.listen(2)
	while True:
		client_socket, address = server_socket.accept()  # accept new connection
		print("Connection from: " + str(address))
  
		# receive writing or reading signal
		signal = client_socket.recv(BUFFER_SIZE).decode()
		print("command: " + signal)
  
		if signal == "write":
			# receive using client socket, not server socket
			file_name = client_socket.recv(BUFFER_SIZE).decode()
			print("file name:" + file_name)
		
			# if signal == "write":
				# start receiving the file from the socket 	     
				# and writing to the file stream
			joint_traj_data = []
			while True:	
				data = client_socket.recv(BUFFER_SIZE)
				if not data:
					break
				joint_traj_data.append(data)

			joint_traj = pickle.loads(b"".join(joint_traj_data))
			with open(file_name, "w") as f:
				for q_point in joint_traj:
					f.write(' '.join('%s' % x for x in list(q_point)))
					# f.write(' ')
					# f.write(' '.join('%s' % x for x in point.velocities))
					f.write(' \n')

		
		elif signal == "read":
			data_folder = client_socket.recv(BUFFER_SIZE).decode()
			print("data folder: " + data_folder)
			data_loader = Dataloader()
			sorted_record_multi = data_loader.load_one_trail(sources=sources, 
											data_folder=data_folder,
											full_range=True, 
											start="sensed", 
											end="ended")
   
			_, state_list = data_loader.get_state_transition()
			state_data = pickle.dumps(state_list)
			client_socket.send(state_data)
	
			sensor_data = pickle.dumps(sorted_record_multi)
			client_socket.sendall(sensor_data)
   
		elif signal == "os":
			data_folder = client_socket.recv(BUFFER_SIZE).decode()
			print("data folder: " + data_folder)
			existing_indexes = os.listdir(data_folder)
			indexes = np.array([eval(i) for i in existing_indexes])
			index_data = pickle.dumps(indexes)
			client_socket.send(index_data)

		# close the client socket
		print("Close connection from: " + str(address))
		client_socket.close()
		
	
 
# 	while True:
# 		# receive data stream. it won't accept data packet greater than 1024 bytes
# 		data = client_socket.recv(8192).decode()
# 		if not data:
# 			# if data is not received break
# 			break
# 		print("from connected user: " + str(data))
# 		data = input(' -> ')
# 		client_socket.send(data.encode())  # send data to the client
  
#   # receive the file infos




# 	conn.close()  # close the connection