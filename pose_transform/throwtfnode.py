import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped

class ThrowTFNode(Node):
    def __init__(self):
        super().__init__('throw_tf_node')

        self.publisher = self.create_publisher(TFMessage, '/tf_2', 10)

    def throw_tf(self, new_tf_message):

        new_tf_message = TFMessage()
        new_transform_stamped = TransformStamped()
        new_transform_stamped.header.frame_id = 'camera_1'
        new_transform_stamped.child_frame_id = 'tagID_1'
        new_transform_stamped.transform.translation.x = 0.3633070657556823
        new_transform_stamped.transform.translation.y = 0.19205179472479567
        new_transform_stamped.transform.translation.z = 1.7602946782879632
        new_transform_stamped.transform.rotation.x = -0.0866144734245208
        new_transform_stamped.transform.rotation.y = -0.954125264648704
        new_transform_stamped.transform.rotation.z = -0.22110438749203234
        new_transform_stamped.transform.rotation.w = 0.18236162475752893
        new_tf_message.transforms.append(new_transform_stamped)
        self.publisher.publish(new_tf_message)
        self.get_logger().info('Published TF message')

def main(args=None):
    rclpy.init(args=args)

    throw_tf_node = ThrowTFNode()

    rclpy.spin(throw_tf_node)

    throw_tf_node.destroy_node()
    rclpy.shutdown()
       