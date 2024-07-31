#import debugpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os
import yaml

# debugpy.listen(("0.0.0.0", 5678))
# print("Waiting for debugger attach...")
# debugpy.wait_for_client()
# print("Debugger attached.")


def generate_launch_description():

    package_dir = get_package_share_directory('pose_transform')
    parameters = os.path.join(package_dir, 'params', 'config.yaml')
    print("Resolved path to config.yaml:", parameters)


    with open(parameters, 'r') as f:
        params = yaml.safe_load(f)['pose_transform_node']['ros__parameters']
    print(params)
        
    frames = params.get('base_frame', [])
    containers = []

    for i, frame in enumerate(frames, start=1):
        print(i, frame)
        container_name = f'PoseTransformNodeContainer{i}'
        node_name = f'pose_transform{i}'

        mapping = [
            ('base_frame', f'{frame}'),
            ('/tf', f'/tf_{i}'),
            ('/tf_modified', f'/pose_{i}')
        ]
    
        
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
                    parameters=[params],
                    remappings=mapping
                )
            ],
            output='screen'
        )
        
        containers.append(container)
    
    
    
    
    
    
    return LaunchDescription(containers)