from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    """
    웨어러블 로봇 시스템의 모든 구성 요소를 시작하는 메인 런치 파일입니다.
    1. CAN 초기화 명령어 자동 실행
    2. ros2_socketcan receiver 런치 파일 실행
    3. data_processing 패키지의 모든 노드 실행
    """

    # 패키지 경로 설정
    socketcan_pkg_share = get_package_share_directory('ros2_socketcan')
    control_pkg_share = get_package_share_directory('wearable_robot_control')
    data_processing_pkg_share = get_package_share_directory('wearable_robot_data_processing')

    socketcan_receiver_launch = os.path.join(
        socketcan_pkg_share,
        'launch',
        'socket_can_receiver.launch.py'  # ros2_socketcan 패키지의 런치 파일명 (실제 파일명에 맞게 수정 필요)
    )

    # 1. CAN 초기화 명령어 실행
    # can0 인터페이스 다운 (만약 이미 실행 중이라면)
    can_down = ExecuteProcess(
        cmd=['sudo ip link set down can0'],
        name='can_down',
        shell=True,
        output='screen'
    )

        # 종료 이벤트 핸들러 등록
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[can_down]
        )
    )

    # can0 인터페이스 설정 (os 부팅 시 실행 됨)
    # can_setup = ExecuteProcess(
    #     cmd=['setup_can'],
    #     name='can_setup',
    #     shell=True,
    #     output='screen'
    # )

    # can0 인터페이스 활성화
    can_up = ExecuteProcess(
        cmd=['sudo ip link set up can0'],
        name='can_up',
        shell=True,
        output='screen'
    )

    # 2. ros2_socketcan 런치 파일 실행 (CAN 설정 완료 후)
    socketcan_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([socketcan_receiver_launch]),
        launch_arguments={
            'interface':'can0',
            'enable_can_fd':'true',
            'interval_sec':'0.012',
            'filters':"0:0"
        }.items()
    )


    # config 파일 경로
    waist_control_config = os.path.join(control_pkg_share, 'config', 'waist_support_params.yaml')
    displacemet_calib_config = os.path.join(data_processing_pkg_share, 'config', 'displacement_calibration_params.yaml')

    # can data processing 노드
    can_processor_node = Node(
        package= 'wearable_robot_data_processing',
        executable='can_data_processor',
        name='can_data_processor',
        output='screen'
    )

    # 데이터 파싱 노드
    parser_node = Node(
        package= 'wearable_robot_data_processing',
        executable='data_parser_node',
        name='data_parser_node',
        output='screen'
    )


    # 변위 데이터 캘리브레이션 노드
    displacement_calib_node = Node(
        package= 'wearable_robot_data_processing',
        executable='displacement_processing_node',
        name='displacement_processing_node',
        parameters=[displacemet_calib_config],
        output='screen'
    )

    # 허리 제어 노드
    wasit_control_node = Node(
        package= 'wearable_robot_control',
        executable='waist_support_control_node',
        name='waist_support_control_node',
        parameters=[waist_control_config],
        output='screen'
    )

    # 구동기 제어 노드
    actuator_control_node = Node(
        package= 'wearable_robot_control',
        executable='waist_actuator_control_node',
        name='waist_actuator_control_node',
        parameters=[waist_control_config],
        output='screen'
    )

    # CAN 송신 노드
    command_send_node = Node(
        package= 'wearable_robot_control',
        executable='can_transmitter_node',
        name='can_transmitter_node',
        parameters=[waist_control_config],
        output='screen'
    )


    # command 노드
    gui_node = Node(
        package= 'wearable_robot_rqt_plugins',
        executable='waist_control_plugin',
        name='waist_control_plugin_launch',
        output='screen',
        on_exit=Shutdown()
    )

    return LaunchDescription([
        # CAN 초기화 명령어 실행
        #can_down,
        #can_setup,
        can_up,
        socketcan_launch_include,
        can_processor_node,
        parser_node,
        displacement_calib_node,
        wasit_control_node,
        actuator_control_node,
        command_send_node,
        gui_node,
        shutdown_handler
    ])
