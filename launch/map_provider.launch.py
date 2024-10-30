import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():
    share_dir = get_package_share_directory('mrg_slam_map_provider')
    config_file = os.path.join(share_dir, 'config', 'map_provider.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)['/**']['ros__parameters']

        # print as defined in the yaml file
        print(f'{config_file}\n{yaml.dump(config, default_flow_style=False, sort_keys=False)}')

    container_name = config['component_container_name']

    # If container_name is empty, we will run the node without a container
    if container_name == '':
        return LaunchDescription([
            Node(
                package='mrg_slam_map_provider',
                executable='map_provider_node',
                name='mrg_slam_map_provider',
                output='screen',
                parameters=[config]
            )
        ])

    # Create the mrg_slam_map_provider component
    mrg_slam_map_provider_component = ComposableNode(
        package='mrg_slam_map_provider',
        plugin='map_provider::MapProviderComponent',
        name='mrg_slam_map_provider',
        namespace='',
        parameters=[config],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    if config['create_own_container']:
        container = ComposableNodeContainer(
            name=container_name,
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[mrg_slam_map_provider_component],
            output='screen',
            parameters=[config]
        )
        container = [container]
    else:
        container = Node(
            package='rclcpp_components',
            executable='component_container',
            name=container_name,
            output='screen',
            parameters=[config]
        )
        load_composable_nodes = LoadComposableNodes(
            composable_node_descriptions=[mrg_slam_map_provider_component],
            target_container=container_name
        )

        container = [load_composable_nodes]

    return LaunchDescription(container)
