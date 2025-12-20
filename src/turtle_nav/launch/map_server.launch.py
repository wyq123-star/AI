from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'yaml_filename': 'src/turtle_nav/maps/maze.yaml'
        }]
    )

    # 1️⃣ 启动后立即 configure
    configure_event = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == map_server,
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    # 2️⃣ 延迟 2 秒 activate（Humble 推荐做法）
    activate_event = TimerAction(
        period=2.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=lambda node: node == map_server,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            )
        ]
    )

    return LaunchDescription([
        map_server,
        configure_event,
        activate_event,
    ])
