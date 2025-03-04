import socket
import numpy as np
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from threading import Lock  # Ensure thread safety
import tf2_ros
import tf2_geometry_msgs
import argparse

class RopeReceiver(Node):
    def __init__(self, name, port, shared_data):
        super().__init__( name +'_receiver')
        
        self.ip = "192.168.1.7"
        self.name = name
        self.shared_data = shared_data
        
        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.ip, port))
        self.get_logger().info(f"Listening for UDP packets on {port}")
        
        # Store received rope positions
        self.data_lock = Lock()  # Prevent concurrency issues

        # Run UDP listener loop in a separate thread
        self.create_timer(0.1, self.receive_and_store)
        
    def receive_and_store(self):
        try:
            # Receive shape metadata first
            # shape_size = np.dtype(np.int32).itemsize * 2  # Two integers for shape (N, 3)
            shape_data, addr = self.sock.recvfrom(1024)
            shape_info = np.frombuffer(shape_data, dtype=np.int32)

            if shape_info.size != 2 or shape_info[1] != 3:
                self.get_logger().error("Invalid shape received")
                return

            num_points = shape_info[0]
            expected_size = num_points * 3 * np.dtype(np.float64).itemsize  # Expected data size

            self.get_logger().info(f"Receiving {num_points} rope points...")

            # Collect multiple packets until we receive all data
            received_data = bytearray()
            while len(received_data) < expected_size:
                chunk, _ = self.sock.recvfrom(8192)  # Receive a chunk
                received_data.extend(chunk)
                self._logger.info(f"Received {len(chunk)} bytes")

            # Convert to numpy array
            rope_positions = np.frombuffer(received_data, dtype=np.float64).reshape(num_points, 3)

            # Store the received rope positions
            with self.data_lock:
                self.shared_data["rope_positions"] = rope_positions.copy() # Update stored positions

            self.get_logger().info(f"Stored {len(rope_positions)} rope points")

        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")
            
class RopeMarkerPublisher(Node):
    def __init__(self, name, shared_data, color):
        super().__init__( name +'_publisher')
        self.name = name
        self.marker_color = color
        
        # Shared memory
        self.shared_data = shared_data
        self.data_lock = Lock()  # Prevent race conditions

        # ROS2 Marker Publisher
        self.topic = "/"+self.name+"_marker"  
        
        #  Initialize TF2 Buffer & Listener once
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=2))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Continuous publishing loop
        if self.name == 'tcp_points':
            self.marker_array_pub = self.create_publisher(MarkerArray, self.topic, 10)
            self.create_timer(0.5, self.publish_point_loop)
        else:
            self.marker_pub = self.create_publisher(Marker, self.topic, 10)
            self.create_timer(0.5, self.publish_line_loop)
        
    def transform_point(self, point, source_frame, target_frame):
        """ Transform a point using TF2, waiting up to 100ms for a valid TF """
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())

            stamped_point = PointStamped()
            stamped_point.header.frame_id = source_frame
            stamped_point.header.stamp = self.get_clock().now().to_msg()
            stamped_point.point.x = point.x
            stamped_point.point.y = point.y
            stamped_point.point.z = point.z

            transformed_point = tf2_geometry_msgs.do_transform_point(stamped_point, transform)
            return transformed_point.point

        except tf2_ros.LookupException:
            self.get_logger().warn(f"TF lookup failed from {source_frame} to {target_frame}. Retrying...")
            return point  # Return original if TF lookup fails

    def publish_line_loop(self):
        """ Continuously publishes the latest stored rope points """
        # self.get_logger().info("Publish loop triggered")  # Debug log
        with self.data_lock:
            rope_positions = self.shared_data.get("rope_positions", np.empty((0, 3), dtype=np.float64))

        with self.data_lock:
            if rope_positions.shape[0] == 0:
                return  # No data to publish yet

            # Create Marker message
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = self.name + "_model"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # Line thickness
            marker.color.r = self.marker_color[0]
            marker.color.g = self.marker_color[1]
            marker.color.b = self.marker_color[2]
            marker.color.a = 1.0
            
            # Add stored points
            for point in rope_positions:
                p = Point(x=point[0], y=point[1], z=point[2])
                transformed_point = self.transform_point(p, "right_panda_link0", "world")
                marker.points.append(transformed_point)

            # Publish continuously
            self.marker_pub.publish(marker)
            self.get_logger().info(f"Published {len(rope_positions)} rope points to {self.topic}")
    
    def publish_line_loop_as_marker(self):
        """ Continuously publishes the latest stored rope points as sphere markers """
        # Retrieve the current rope positions safely
        with self.data_lock:
            rope_positions = self.shared_data.get("rope_positions", np.empty((0, 3), dtype=np.float64))

        # If there are no points, exit the function
        with self.data_lock:
            if rope_positions.shape[0] == 0:
                return  # No data to publish yet

            # Create a Marker message with SPHERE_LIST type
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = self.name + "_model"
            marker.id = 0
            marker.type = Marker.SPHERE_LIST  # Use SPHERE_LIST instead of LINE_STRIP
            marker.action = Marker.ADD
            
            # Set the scale for spheres (all dimensions should be set)
            sphere_diameter = 0.015  # Adjust this value to change the sphere size
            marker.scale.x = sphere_diameter
            marker.scale.y = sphere_diameter
            marker.scale.z = sphere_diameter
            
            # Set marker color
            marker.color.r = self.marker_color[0]
            marker.color.g = self.marker_color[1]
            marker.color.b = self.marker_color[2]
            marker.color.a = 1.0
            
            # Add each point as a sphere marker
            for i, point in enumerate(rope_positions):
                if i % 2 == 0:
                    p = Point(x=point[0], y=point[1], z=point[2])
                    transformed_point = self.transform_point(p, "right_panda_link0", "world")
                    marker.points.append(transformed_point)
            
            # Publish the sphere markers
            self.marker_pub.publish(marker)
            self.get_logger().info(f"Published {len(rope_positions)} rope points as sphere markers to {self.topic}")

    def publish_point_loop(self):
        """ Continuously publishes the latest stored rope points """
        with self.data_lock:
            rope_positions = self.shared_data.get("rope_positions", np.empty((0, 3), dtype=np.float64))

        with self.data_lock:
            if rope_positions.shape[0] == 0:
                return  # No data to publish yet

            marker_array = MarkerArray()

            for i, point in enumerate(rope_positions):
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = self.name + "_model"
                marker.id = i  # Unique ID for each marker
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale.x = 0.03  # Diameter of the ball
                marker.scale.y = 0.03
                marker.scale.z = 0.03
                marker.color.r = self.marker_color[0]
                marker.color.g = self.marker_color[1]
                marker.color.b = self.marker_color[2]
                marker.color.a = 1.0

                # Set the position of the ball marker
                p = Point(x=point[0], y=point[1], z=point[2])
                transformed_point = self.transform_point(p, "right_panda_link0", "world")
                marker.pose.position = transformed_point

                marker_array.markers.append(marker)

            # Publish the marker array
            self.marker_array_pub.publish(marker_array)
            self.get_logger().info(f"Published tcp at position {rope_positions[2]} to {self.topic}")

def main(args=None):
    parser = argparse.ArgumentParser(description='Rope Marker Publisher')
    parser.add_argument('name', type=str, help='Type of the rope data')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    
    raw_port = 5090
    corrected_port = 5093
    intersection_port = 5092
    
    # Shared memory for rope positions
    shared_data = {"rope_positions": np.empty((0, 3), dtype=np.float64)}

    if parsed_args.name == 'raw':
        node_name = 'raw_rope'
        port = raw_port
        color = [0.0, 1.0, 0.0] # green
    elif parsed_args.name == 'corrected':
        node_name = 'corrected_rope'
        port = corrected_port
        color = [1.0, 0.0, 0.0] # red
    elif parsed_args.name == 'tcp':
        node_name = 'tcp_points'
        port = intersection_port
        color = [0.0, 0.0, 1.0]
    else:
        raise ValueError("Invalid rope type")
    
    receiver_nod = RopeReceiver(node_name, port, shared_data)
    publisher_node = RopeMarkerPublisher(node_name, shared_data, color)
    
    # Use MultiThreadedExecutor to handle multiple nodes
    executor = MultiThreadedExecutor()
    executor.add_node(receiver_nod)
    executor.add_node(publisher_node)

    try:
        executor.spin() 
    except KeyboardInterrupt:
        pass
    finally:
        # raw_node.destroy_node()
        receiver_nod.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()
        

if __name__ == "__main__":
    main()
