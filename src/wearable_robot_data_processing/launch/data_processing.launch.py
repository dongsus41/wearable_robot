from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    wearable_robot_data_processing 패키지의 테스트를 위한 launch 파일
    반드시 CAN 인터페이스 설정 선행 후 실행해야함!
    """

    # 패키지 경로 가져오기
    data_processing_share = get_package_share_directory('wearable_robot_data_processing')

    # 설정 파일 경로
    config_file = os.path.join(data_processing_share, 'config', 'displacement_calibration_params.yaml')

    # 데이터 처리 패키지의 노드들
    can_data_processor = Node(
        package='wearable_robot_data_processing',
        executable='can_data_processor',
        name='can_data_processor',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    data_parser_node = Node(
        package='wearable_robot_data_processing',
        executable='data_parser_node',
        name='data_parser_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    displacement_processing_node = Node(
        package='wearable_robot_data_processing',
        executable='displacement_processing_node',
        name='displacement_processing_node',
        output='screen',
        parameters=[
            config_file,  # 직접 설정 파일 경로 전달
            {'use_sim_time': False}
        ]
    )

    imu_processing_node = Node(
        package='wearable_robot_data_processing',
        executable='IMU_processing_node',
        name='imu_processing_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 모든 구성요소를 LaunchDescription에 추가
    return LaunchDescription([
        # 데이터 처리 노드들
        can_data_processor,
        data_parser_node,
        displacement_processing_node,
        imu_processing_node,
    ])
