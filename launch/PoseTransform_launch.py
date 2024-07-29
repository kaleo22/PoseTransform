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


    with open(parameters, 'r') as f:
        params = yaml.safe_load(f)['pose_transform_node']['ros__parameters']
    print(params)
        
    frames = params.get('base_frame', [])
    containers = []

    for i in enumerate(frames, start=1):
        container_name = f'PoseTransformNodeContainer{i}'
        node_name = f'pose_transform{i}'
        
        container = ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='pose_transform',
            executable='PoseTransformNode',
            composable_node_descriptions=[
                ComposableNode(
                    package='pose_transform',
                    plugin='',
                    name=node_name,
                    parameters=[params])
            ],
            output='screen'
        )
        
        containers.append(container)
    
    
    
    
    
    
    return LaunchDescription(containers)