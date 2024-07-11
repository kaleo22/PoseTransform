#!/home/leonard/peak_cam_docker_container/.venv/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PointStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

class PoseTransposeNode(Node):

    def __init__(self):
        super().__init__('pose_transpose_node')
        
        # Declare parameters
        self.declare_parameter('base_frame', [])
        self.declare_parameter('target_frames', [])
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.target_frames = self.get_parameter('target_frames').get_parameter_value().string_array_value
        
        # Create a buffer and a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer to periodically check for transforms
        self.timer = self.create_timer(0.1, self.on_timer)
        
        # Publisher for the /pose topic
        self.pose_publisher = self.create_publisher(PointStamped, '/pose', 10)
        
        self.origin = None

    def on_timer(self):
        for target_frame in self.target_frames:
            # Try to get the latest transform
            try:
                trans = self.tf_buffer.lookup_transform(self.base_frame, target_frame, Time())
                self.process_transform(trans, target_frame)
            except Exception as e:
                self.get_logger().info(f'Could not get transform from {self.base_frame} to {target_frame}: {e}')

    def process_transform(self, trans: TransformStamped, target_frame: str):
        # Extract translation
        translation = trans.transform.translation
        x = translation.x
        y = translation.y
        z = translation.z
        
        # Extract rotation
        rotation = trans.transform.rotation
        rx = rotation.x
        ry = rotation.y
        rz = rotation.z
        rw = rotation.w

        if self.origin:
            rot = R.from_quat([rw, rx, ry, rz])
            trans_vec = np.array([x, y, z])
            pose = rot.apply(trans_vec) - self.origin
            x, y, z = pose[0], pose[1], pose[2]
        elif target_frame == 'tagID_0' and self.origin is None:
            rot = R.from_quat([rw, rx, ry, rz])
            trans_vec = np.array([x, y, z])
            self.origin = rot.apply(trans_vec)

        # Log the results
        self.get_logger().info(f'Transform from {self.base_frame} to {target_frame}:')
        self.get_logger().info(f'Translation: x={x}, y={y}, z={z}')
        self.get_logger().info(f'Rotation: x={rx}, y={ry}, z={rz}, w={rw}')
        
        # Publish the pose
        pose_msg = PointStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = target_frame
        pose_msg.point.x = x
        pose_msg.point.y = y
        pose_msg.point.z = z
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node and load parameters
    node = PoseTransposeNode()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
