from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    # =========================
    # 1. turtlesim 本体
    # =========================
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    # =========================
    # 2. 静态地图节点
    # =========================
    static_map_node = Node(
        package='turtle_nav',
        executable='turtle_static_map',
        name='turtle_static_map',
        output='screen'
    )

    # =========================
    # 3. TF + odom 适配节点
    # =========================
    turtle_tf_node = Node(
        package='turtle_nav',
        executable='turtle_tf',
        name='turtle_tf',
        output='screen'
    )

    # =========================
    # 4. RViz2 节点
    # =========================
    rviz_config = os.path.join(
        get_package_share_directory('turtle_nav'),
        'rviz',
        'turtle_nav.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    astar_node = Node(
        package='turtle_nav',
        executable='turtle_astar_planner',
        name='turtle_astar_planner',
        output='screen'
    )

    ld.add_action(astar_node)
    ld.add_action(turtlesim_node)
    ld.add_action(static_map_node)
    ld.add_action(turtle_tf_node)
    ld.add_action(rviz_node)

    return ld
