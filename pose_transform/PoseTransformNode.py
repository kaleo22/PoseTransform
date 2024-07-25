#!/home/leonard/peak_cam_docker_container/.venv/bin/python3


import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped

class Pose_Transform_Node(Node):
    def __init__(self):
        super().__init__('pose_transform_node')

        self.Origin = np.array([-0.20203712, 0.36291002, -1.49017612])

        self.declare_parameter('base_frame', 'default_base_frame')
        self.declare_parameter('target_frame', ['default_target_frame'])

        self.base_frame = self.get_parameter('base_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(TFMessage, '/tf_modified', 10)
        self.x = self.y = self.z = 0.0
        self.rx = self.ry = self.rz = self.rw = 0.0


    def listener_callback(self, msg):
        
        new_tf_message = TFMessage()
        
        for transform_stamped in msg.transforms:
            if transform_stamped.header.frame_id == 'camera_1' and transform_stamped.child_frame_id == 'tagID_0' and self.Origin is None:
                
                self.frame_id = transform_stamped.header.frame_id
                self.child_frame_id = transform_stamped.child_frame_id
                self.x = transform_stamped.transform.translation.x
                self.y = transform_stamped.transform.translation.y
                self.z = transform_stamped.transform.translation.z
                self.rx = transform_stamped.transform.rotation.x
                self.ry = transform_stamped.transform.rotation.y
                self.rz = transform_stamped.transform.rotation.z
                self.rw = transform_stamped.transform.rotation.w

                rot = R.from_quat([self.rx, self.ry, self.rz, self.rw])
                rot_inv = rot.inv()
                trans_vec = np.array([self.x, self.y, self.z])
                pose = rot_inv.apply(trans_vec)
                self.x, self.y, self.z = pose[0], pose[1], pose[2]
                self.Origin = np.array([self.x, self.y, self.z])

                
                new_transform_stamped = TransformStamped()
                
                
                new_transform_stamped.header.frame_id = self.frame_id
                new_transform_stamped.header.stamp = transform_stamped.header.stamp  
                new_transform_stamped.child_frame_id = self.child_frame_id
                
                # Set the new translation and rotation values
                new_transform_stamped.transform.translation.x = self.x
                new_transform_stamped.transform.translation.y = self.y
                new_transform_stamped.transform.translation.z = self.z
                new_transform_stamped.transform.rotation.x = self.rx
                new_transform_stamped.transform.rotation.y = self.ry
                new_transform_stamped.transform.rotation.z = self.rz
                new_transform_stamped.transform.rotation.w = self.rw

                new_tf_message.transforms.append(new_transform_stamped)

            else:
                self.frame_id = transform_stamped.header.frame_id
                self.child_frame_id = transform_stamped.child_frame_id
                self.x = transform_stamped.transform.translation.x
                self.y = transform_stamped.transform.translation.y
                self.z = transform_stamped.transform.translation.z
                self.rx = transform_stamped.transform.rotation.x
                self.ry = transform_stamped.transform.rotation.y
                self.rz = transform_stamped.transform.rotation.z
                self.rw = transform_stamped.transform.rotation.w

                self.rot = R.from_quat([self.rx, self.ry, self.rz, self.rw])
                self.rot_inv = self.rot.inv()
                self.trans_vec = np.array([self.x, self.y, self.z])
                self.pose = self.rot_inv.apply(self.trans_vec) 
                self.newpose = (self.pose - self.Origin) * 1.638361
                self.get_logger().info(f"Pose: {self.pose}")
                self.get_logger().info(f"New Pose: {self.newpose}")
                self.x, self.y, self.z = self.newpose[0], self.newpose[1], self.newpose[2]

                
                self.new_transform_stamped = TransformStamped()
                
                
                self.new_transform_stamped.header.frame_id = self.frame_id
                self.new_transform_stamped.header.stamp = transform_stamped.header.stamp  
                self.new_transform_stamped.child_frame_id = self.child_frame_id
                
                # Set the new translation and rotation values
                self.new_transform_stamped.transform.translation.x = self.x
                self.new_transform_stamped.transform.translation.y = self.y
                self.new_transform_stamped.transform.translation.z = self.z
                self.new_transform_stamped.transform.rotation.x = self.rx
                self.new_transform_stamped.transform.rotation.y = self.ry
                self.new_transform_stamped.transform.rotation.z = self.rz
                self.new_transform_stamped.transform.rotation.w = self.rw

                new_tf_message.transforms.append(self.new_transform_stamped)

            
                
                self.get_logger().info(f"Frame ID: {self.new_transform_stamped.header.frame_id}")
                self.get_logger().info(f"Frame ID: {self.new_transform_stamped.child_frame_id}")
                
                self.get_logger().info(f"New TF Message: {new_tf_message}")
                self.publisher.publish(new_tf_message)

def main(args=None):
    rclpy.init(args=args)
    tf_listener_publisher = Pose_Transform_Node()
    rclpy.spin(tf_listener_publisher)
    tf_listener_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
