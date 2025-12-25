from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # =========================
    # turtlesim
    # =========================
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    # =========================
    # TF
    # =========================
    turtle_tf_node = Node(
        package='turtle_nav',
        executable='turtle_tf',
        name='turtle_tf',
        output='screen'
    )

    # =========================
    # A*
    # =========================
    astar_node = Node(
        package='turtle_nav',
        executable='turtle_astar_planner',
        parameters=[{
            'use_inflation': True,
            'use_costmap': True,
            'inflation_radius': 0.25,
            'cost_scaling': 8.0
        }]
    )


    # =========================
    # follower
    # =========================
    follower_node = Node(
        package='turtle_nav',
        executable='turtle_path_follower',
        name='turtle_path_follower',
        output='screen'
    )

    # =========================
    # map_server launch
    # =========================
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtle_nav'),
                'launch',
                'map_server.launch.py'
            )
        )
    )

    dynamic_obstacle_publisher_node = Node(
        package='turtle_nav',
        executable='dynamic_obstacle_publisher',
        name='dynamic_obstacle_publisher',
        output='screen'
    )

    map_fusion_node = Node(
        package='turtle_nav',
        executable='map_fusion_node',
        name='map_fusion_node',
        output='screen'
    )

    obstacle_motion = Node(
        package='turtle_nav',
        executable='obstacle_motion',
        name='obstacle_motion',
        output='screen'
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_tf_node)
    ld.add_action(astar_node)
    ld.add_action(map_server_launch)
    # ld.add_action(follower_node)
    ld.add_action(dynamic_obstacle_publisher_node)
    ld.add_action(obstacle_motion)
    # ld.add_action(map_fusion_node)
    return ld
