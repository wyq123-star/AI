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

    # =========================
    # 启动顺序控制
    # =========================

    # # A* 启动 → 启动 map_server
    # start_map_after_astar = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=astar_node,
    #         on_start=[map_server_launch]
    #     )
    # )

    # # map_server 启动 → 启动 follower
    # start_follower_after_map = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=map_server_launch,
    #         on_start=[follower_node]
    #     )
    # )

    # =========================
    # 添加到 LaunchDescription
    # =========================
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_tf_node)
    ld.add_action(astar_node)
    # ld.add_action(map_server_launch)
    ld.add_action(follower_node)
    return ld
