# udp_receiver.py

import socket
import json

class UDPReceiver:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.buffer_size = 4096  # Adjust as needed
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        print(f"Listening on {self.host}:{self.port}")

    def receive_single_data(self, data_type):
        """
        Receive a single data packet and return the processed result.
        """
        try:
            # Receive a single message and process it
            data, addr = self.sock.recvfrom(self.buffer_size)
            message = data.decode()
            print(f"Received message from {addr}: {message}")
            return self.process_message(data_type, message)
        except OSError as e:
            # Handle the Bad File Descriptor error by reinitializing the receiver
            if e.errno == 9:  # errno 9 corresponds to "Bad file descriptor"
                print(f"Socket error detected: {e}. Reinitializing receiver...")
                self.reinitialize_receiver()
            else:
                print(f"Unexpected socket error: {e}")
            return None
        return data

    def process_message(self, data_type, message):
        """
        Process a single message and return the relevant data based on the type.
        """
        try:
            message_dict = json.loads(message)
            data = message_dict.get("data")

            if data_type == "intersections":
                return self.get_tactile_intersections(data)
            if data_type == "init_grasp_position":
                return self.get_init_grasp_position(data, data_type)
            if data_type == "init_grasp_position2":
                return self.get_init_grasp_position(data, data_type)
            else:
                print(f"Unknown message type: {data_type}")
                return None
        except json.JSONDecodeError:
            print("Failed to decode JSON message.")
            return None

    def get_tactile_intersections(self, data):
        """
        Extract intersection points from the received data.
        """
        intersection_points = []
        intersections = data.get("intersections", [])
        print("Tactile intersections data:")
        
        # Extract x and y coordinates from each point in intersections
        for point in intersections:
            x = point.get("x")
            y = point.get("y")
            intersection_points.append((x, y))
        
        print(f" - Intersections: {intersection_points}")
        return intersection_points

    def get_init_grasp_position(self, data, data_type):
        """
        Extract init_grasp_position from the received data.
        """
        # The `data` should already be a dictionary, so directly access its keys
        init_grasp_position = data.get(data_type, {})
        print(data_type)

        # Extract x, y, and z from the init_grasp_position dictionary
        x = init_grasp_position.get("x")
        y = init_grasp_position.get("y")
        z = init_grasp_position.get("z")

        # Print the 3D point
        print(f" - x: {x}, y: {y}, z: {z}")
        
        # Return as a dictionary
        return {'x': x, 'y': y, 'z': z}
    
    # def send_data(self, udp_host, udp_port, data):
    #     """
    #     Sends the given data to the specified UDP host and port.
    #     """
    #     # Create a UDP client socket
    #     sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    #     # Convert the data to a JSON string
    #     message = json.dumps(data)
        
    #     # Send the data
    #     sock.sendto(message.encode(), (udp_host, udp_port))
    #     print(f"Sent data to {udp_host}:{udp_port} - {message}")
        
    #     # Close the socket
    #     sock.close()

    def send_data(self, intersection3d1, intersection3d2, tcp, udp_host, udp_port):
        # Prepare data as a single list with 9 numbers
        data = intersection3d1.tolist() + intersection3d2.tolist() + tcp

        # Send the plain list
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # sock.sendto(str(data).encode(), (udp_host, udp_port))
        message = json.dumps(data).encode('utf-8')
        sock.sendto(message, (udp_host, udp_port))
        sock.close()
        print(f"Sent data: {data}")

    def reinitialize_receiver(self):
        """
        Reinitialize the socket and buffer if a 'Bad file descriptor' error is detected.
        """
        # Close the existing socket if it's in an invalid state
        if self.sock:
            try:
                self.sock.close()
                print("Closed the invalid socket.")
            except OSError as e:
                print(f"Error closing socket: {e}")

        # Re-create and bind a new socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        print(f"Socket reinitialized and listening on {self.host}:{self.port}")