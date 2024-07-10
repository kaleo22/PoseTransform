import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import numpy as np

class TFListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener_node')
        
        # Declare parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('target_frames', ['target_frame'])
        
        # Get parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.target_frames = self.get_parameter('target_frames').get_parameter_value().string_array_value
        
        # Create a buffer and a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer to periodically check for transforms
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        for target_frame in self.target_frames:
            # Try to get the latest transform
            try:
                trans = self.tf_buffer.lookup_transform(self.base_frame, target_frame, rclpy.time.Time())
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


        try: 

            if Origin: 

                rot = R.from_quat(rw, rx, ry, rz)

                trans = np.array([x], [y], [z])

                Pose = rot.apply(trans) - Origin

                x = Pose[1, 1]
                y = Pose[2, 1]
                z = Pose[3, 1]

        except NameError:

            self.get_logger().info(f'Tag0 not found!')
        

        if target_frame == str('Tag0') and Origin not in locals() and Origin not in globals():
            rot = R.from_quat(rw, rx, ry, rz)

            trans = np.array([x], [y], [z])

            Pose = rot.apply(trans)

            Origin = Pose
        
        else:
            
            pass
        
        # Log the results
        self.get_logger().info(f'Transform from {self.base_frame} to {target_frame}:')
        self.get_logger().info(f'Translation: x={x}, y={y}, z={z}')
        self.get_logger().info(f'Rotation: x={rx}, y={ry}, z={rz}, w={rw}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node and load parameters
    node = TFListenerNode()
    
    # Load parameters from a YAML file
    param_file = rclpy.utilities.get_parameters_file_path()
    node.declare_parameters_from_file(param_file)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

