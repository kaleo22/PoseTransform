#!/home/leonard/peak_cam_docker_container/.venv/bin/python3


import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import numpy as np
from scipy.spatial.transform import Rotation as R

class TFListenerPublisher(Node):
    def __init__(self):
        super().__init__('pose_transform_node')

        self.declare_parameter('base_frame', 'default_base_frame')
        self.declare_parameter('target_frame', ['default_target_frame'])

        self.base_frame = self.get_parameter('base_frame').get_value()
        self.target_frame = self.get_parameter('target_frame').get_value()

        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(TFMessage, '/tf_modified', 10)
        self.x = self.y = self.z = 0.0
        self.rx = self.ry = self.rz = self.rw = 0.0

    def listener_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'camera_1' and transform.child_frame_id == 'tagID_0' and self.Origin is None:
                transform = msg.transforms[0].transform
                self.frame_id = transform.header.frame_id
                self.child_frame_id = transform.child_frame_id
                self.x = transform.translation.x
                self.y = transform.translation.y
                self.z = transform.translation.z
                self.rx = transform.rotation.x
                self.ry = transform.rotation.y
                self.rz = transform.rotation.z
                self.rw = transform.rotation.w

                rot = R.from_quat([self.rw, self.rx, self.ry, self.rz])
                trans_vec = np.array([self.x, self.y, self.z])
                pose = rot.apply(trans_vec)
                self.x, self.y, self.z = pose[0], pose[1], pose[2]
                self.Origin = [self.x, self.y, self.z]

            elif transform.header.frame_id == 'camera_1' and transform.child_frame_id == 'tagID_1' and self.Origin is not None:
                transform = msg.transforms[0].transform
                self.frame_id = transform.header.frame_id
                self.child_frame_id = transform.child_frame_id
                self.x = transform.translation.x
                self.y = transform.translation.y
                self.z = transform.translation.z
                self.rx = transform.rotation.x
                self.ry = transform.rotation.y
                self.rz = transform.rotation.z
                self.rw = transform.rotation.w

                rot = R.from_quat([self.rw, self.rx, self.ry, self.rz])
                trans_vec = np.array([self.x, self.y, self.z])
                pose = rot.apply(trans_vec) - self.Origin
                self.x, self.y, self.z = pose[0], pose[1], pose[2]

            elif transform.header.frame_id == 'camera_1' and transform.child_frame_id == 'tagID_2' and self.Origin is not None:
                transform = msg.transforms[0].transform
                self.frame_id = transform.header.frame_id
                self.child_frame_id = transform.child_frame_id
                self.x = transform.translation.x
                self.y = transform.translation.y
                self.z = transform.translation.z
                self.rx = transform.rotation.x
                self.ry = transform.rotation.y
                self.rz = transform.rotation.z
                self.rw = transform.rotation.w

                rot = R.from_quat([self.rw, self.rx, self.ry, self.rz])
                trans_vec = np.array([self.x, self.y, self.z])
                pose = rot.apply(trans_vec) - self.Origin
                self.x, self.y, self.z = pose[0], pose[1], pose[2]

            else:
                self.get_logger().info('No transform found')
                

                # Here you can do your calculations and modify the x, y, z, rx, ry, rz, rw variables

                # After calculations, create a new TFMessage to publish
                new_tf_message = TFMessage()
                for transform in msg.transforms:
                    new_msg = transform  # Create a new transform message
                    new_msg.transforms[0].header.frame_id = self.frame_id
                    new_msg.transforms[0].child_frame_id = self.child_frame_id
                    new_msg.transforms[0].transform.translation.x = self.x
                    new_msg.transforms[0].transform.translation.y = self.y
                    new_msg.transforms[0].transform.translation.z = self.z
                    new_msg.transforms[0].transform.rotation.x = self.rx
                    new_msg.transforms[0].transform.rotation.y = self.ry
                    new_msg.transforms[0].transform.rotation.z = self.rz
                    new_msg.transforms[0].transform.rotation.w = self.rw

                    # Publish the new TFMessage
                    new_tf_message.transforms.append(new_msg)
                self.publisher.publish(new_tf_message)

def main(args=None):
    rclpy.init(args=args)
    tf_listener_publisher = TFListenerPublisher()
    rclpy.spin(tf_listener_publisher)
    tf_listener_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
