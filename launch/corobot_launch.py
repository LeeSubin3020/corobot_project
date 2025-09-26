from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 웨이크업 키워드 노드
        Node(
            package='corobot2_project',
            executable='wakeup_word_node',
            name='wakeup_word_node',
            output='screen',
        ),
        # STT + 페인트 명령 파서 서비스 서버
        Node(
            package='corobot2_project',
            executable='paint_command_server',
            name='paint_command_server',
            output='screen',
        ),
        # 카메라 색상 매칭 GUI
        Node(
            package='corobot2_project',
            executable='color_matcher_node',
            name='color_matcher_node',
            output='screen',
        ),
        # 페인트 픽커 (YOLO + 로봇 제어)
        Node(
            package='corobot2_project',
            executable='paint_picker_node',
            name='paint_picker_node',
            output='screen',
        ),
    ])
