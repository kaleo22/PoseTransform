from setuptools import setup

package_name = 'pose_transform'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/PoseTransform_launch.py'] ),
        ('share/' + package_name + '/params', ['params/config.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leonard',
    maintainer_email='leonard.kaempf@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'PoseTransformNode = pose_transform.PoseTransformNode:main'
        ],
    },
    
)
