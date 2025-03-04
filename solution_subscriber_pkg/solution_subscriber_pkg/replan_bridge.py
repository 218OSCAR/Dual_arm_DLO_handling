#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import socket
import json
import threading

class ReplanMiddleLayer(Node):
    def __init__(self):
        super().__init__('replan_middle_layer')
        # UDP settings
        self.udp_port = 5087  # Listening port for replan requests
        self.buffer_size = 1024
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Bind to all interfaces on the designated port.
        self.sock.bind(('192.168.1.7', self.udp_port))
        self.get_logger().info(f'UDP server listening on 192.168.1.7:{self.udp_port}')

        # Create a service client for the replan service.
        self.cli = self.create_client(SetBool, 'replan_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for replan_service...')
        self.get_logger().info('Connected to replan_service.')

        # Start UDP listener in a separate thread.
        self.udp_thread = threading.Thread(target=self.udp_listen, daemon=True)
        self.udp_thread.start()

    def udp_listen(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(self.buffer_size)
                msg_str = data.decode('utf-8').strip()
                self.get_logger().info(f"Received UDP message from {addr}: {msg_str}")
                # Parse the incoming data as JSON.
                try:
                    data_json = json.loads(msg_str)
                    # Check for the expected command and use the "replan" field value.
                    if data_json.get("command", "").lower() == "replan":
                        replan_value = data_json.get("replan", False)
                        self.get_logger().info(f"Replan field value: {replan_value}")
                        self.call_replan_service(addr, replan_value)
                    else:
                        self.get_logger().warn(f"Unknown command in UDP message: {data_json}")
                except json.JSONDecodeError as e:
                    self.get_logger().error(f"Failed to decode JSON: {e}")
            except Exception as e:
                self.get_logger().error(f"UDP server error: {e}")

    def call_replan_service(self, addr, replan_value: bool):
        # Prepare and send a SetBool request using the "replan" field value.
        req = SetBool.Request()
        req.data = replan_value
        self.get_logger().info(f"Calling replan_service with data {req.data}")
        future = self.cli.call_async(req)
        
        # Instead of spin_until_future_complete, use a loop with spin_once.
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Replan service response: {response.message}")
            reply = json.dumps({"replan": response.success})
        else:
            self.get_logger().error("Replan service call failed")
            reply = json.dumps({"replan": False})
        
        # Send the JSON reply back to the sender's address.
        try:
            self.sock.sendto(reply.encode('utf-8'), addr)
            self.get_logger().info(f"Sent UDP reply: {reply} to {addr}")
        except Exception as e:
            self.get_logger().error(f"Error sending UDP reply: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ReplanMiddleLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ReplanMiddleLayer")
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
