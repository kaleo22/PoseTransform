from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    
    package_dir = get_package_share_directory('pose_transform')
    parameters = os.path.join(package_dir, 'params', 'config.yaml')
    print("Resolved path to config.yaml:", parameters)

    #parameters = Path('pose_transform/params/config.yaml')

    with open(parameters, 'r') as f:
        params = yaml.safe_load(f)['pose_transform_node']['ros__parameters']
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