from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from pathlib import Path
import yaml

def generate_launch_description():
    
    parameters = Path('/pose_transform/params/config.yaml')

    with open(parameters, 'r') as f:
        params = yaml.safe_load(f)['pose_transpose_node']['ros__parameters']
    print(params)
        
        
    PoseTransformNodeContainer = ComposableNodeContainer(
        name = 'PoseTransformNode',
        namespace = '',
        package = 'pose_transform',
        executable = 'PoseTransformNode',
        composable_node_descriptions=[
            ComposableNode(
                package='pose_transform',
                plugin='',
                name='pose_transform',
                parameters=[params])
        ],
        output = 'screen'
    )
    
    
    
    
    
    
    return LaunchDescription([PoseTransformNodeContainer])