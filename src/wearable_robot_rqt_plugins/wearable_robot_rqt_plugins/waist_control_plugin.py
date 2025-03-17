#!/usr/bin/env python3

import os
import threading
import subprocess
import signal
import datetime
import rclpy
import sys
import rclpy.task
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from wearable_robot_interfaces.msg import ActuatorCommand, TemperatureData, DisplacementData, BackIntension
from wearable_robot_interfaces.srv import SetControlMode, SetControlParams, EmergencyStop
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox, QVBoxLayout, QLabel
from python_qt_binding.QtCore import Qt, QTimer
import std_msgs.msg
import pyqtgraph as pg


class WaistControlPlugin(Plugin):
    """
    웨어러블 로봇의 허리 보조 제어 및 모니터링을 위한 rqt 플러그인
    """
    def __init__(self, context):
        super(WaistControlPlugin, self).__init__(context)
        # 플러그인 제목 설정
        self.setObjectName('WaistControlPlugin')

        # 중요: 클래스 속성 초기화를 먼저 진행
        # 그래프 플로팅 관련 변수 초기화
        self.is_plotting = False            # 그래프 플로팅 상태
        self.buffer_size = 300              # 5분 (300초) 동안의 데이터 저장
        self.time_data = []                 # 시간 데이터 (x축)
        self.temp_data_4 = []               # 4번 구동기 온도 데이터
        self.temp_data_5 = []               # 5번 구동기 온도 데이터
        self.displacement_data = []         # 변위 센서 데이터
        self.target_temp_data = []          # 목표 온도 데이터
        self.threshold_data = []            # 임계값 데이터 (변위 센서용)
        self.pwm_data_4 = []                # 4번 구동기 PWM 데이터
        self.pwm_data_5 = []                # 5번 구동기 PWM 데이터
        self.start_time = 0.0               # 그래프 시작 시간

        # 내부 상태 변수 초기화
        self.temperatures = [0.0] * 6       # 모든 구동기 온도
        self.pwm_values = [0] * 6           # 모든 구동기 PWM 값
        self.displacement = 0.0             # 변위 센서 값
        self.current_intention = 0          # 현재 의도 ID (0: 없음, 1: 있음)
        self.is_emergency_stop = False      # 비상 정지 상태
        self.safety_temp_threshold = 80.0   # 안전 온도 임계값 (°C)
        self.recording_process = None
        self.applied_angle_threshold = 2.0  # 허리 각도 임계값
        self.applied_target_temp = 60.0     # 기본 목표 온도
        self.is_active = False

        # 그래프 관련 객체 초기화
        self.displacement_plot_widget = None
        self.temp_plot_widget = None
        self.pwm_plot_widget = None
        self.displacement_line = None
        self.threshold_line = None
        self.temp_line_4 = None
        self.temp_line_5 = None
        self.target_temp_line = None
        self.pwm_line_4 = None
        self.pwm_line_5 = None
        self.temp_legend = None

        # ROS 노드 초기화
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init()

            node_name = "waist_control_plugin_internal"
            self.node = Node(node_name)

            self.node.get_logger().info(f"노드 생성 성공: {node_name}")

            # QoS 설정
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        except Exception as e:
            print(f"[ERROR]: 노드 생성 실패: {str(e)}")
            return

        # 발행자 생성
        self.target_temp_pub = self.node.create_publisher(
            std_msgs.msg.Float64, 'target_temperature_setting', qos_profile)  # 상위 제어기로 목표 온도 설정
        self.intention_pub = self.node.create_publisher(
            BackIntension, 'intention_data', qos_profile)  # 의도 데이터 발행

        # 구독자 생성
        self.temp_sub = self.node.create_subscription(
            TemperatureData, 'temperature_data', self.temp_callback, qos_profile)  # 온도 데이터
        self.pwm_sub = self.node.create_subscription(
            ActuatorCommand, 'pwm_state', self.pwm_callback, qos_profile)  # 현재 PWM 상태
        self.displacement_sub = self.node.create_subscription(
            DisplacementData, 'displacement_data', self.displacement_callback, qos_profile)  # 변위 데이터

        # 서비스 클라이언트 생성
        self.control_mode_client = self.node.create_client(
            SetControlMode, 'set_control_mode')
        self.pi_params_client = self.node.create_client(
            SetControlParams, 'set_pi_parameters')
        self.emergency_client = self.node.create_client(
            EmergencyStop, 'set_emergency_stop')

        # 서비스 가용성 확인을 위한 타이머
        self.service_check_timer = self.node.create_timer(1.0, self.check_service_availability)
        self.service_availability = {
            'set_control_mode': False,
            'set_pi_parameters': False,
            'set_emergency_stop': False
        }

        # 주기적인 작업을 위한 타이머 (20ms 마다 안전 검사 및 상태 체크)
        self.timer = self.node.create_timer(0.02, self.check_state)

        # 그래프 데이터 수집 타이머 (50Hz, 20ms 간격)
        self.data_collection_timer = self.node.create_timer(0.02, self.collect_data_for_graph)

        self._shutdown_flag = False

        # ROS 2 스핀을 위한 스레드 생성
        self.spin_thread = threading.Thread(target=self.spin_ros)
        self.spin_thread.daemon = True
        self.spin_thread.start()

        # 클래스 내 __init__에 추가
        self.data_lock = threading.RLock()  # 재진입 가능한 락 사용

        # UI 설정
        self._widget = QWidget()

        # UI 파일 경로를 찾기 위한 여러 가능한 위치를 시도
        ui_file = os.path.expanduser("~/wearable_robot_ws/src/wearable_robot_rqt_plugins/resource/waist_control.ui")

        if ui_file is None:
            raise FileNotFoundError("waist_control.ui 파일을 찾을 수 없습니다. 설치가 올바르게 되었는지 확인하세요.")

        loadUi(ui_file, self._widget)
        self._widget.setObjectName('WaistControlPluginUI')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # UI 이벤트 연결
        self._widget.apply_button.clicked.connect(self.apply_settings)
        self._widget.emergency_stop_button.clicked.connect(self.emergency_stop)
        self._widget.intention_button.clicked.connect(self.toggle_intention)

        # 데이터 로깅 경로 설정
        self.data_logging_path = os.path.expanduser("~/WR_logs")

        # 로그 저장 경로가 없으면 생성
        if not os.path.exists(self.data_logging_path):
            os.makedirs(self.data_logging_path, exist_ok=True)

        # 그래프 UI 설정 및 초기화
        self.setup_graphs()

        # 그래프 제어 버튼 연결
        self._widget.graph_start_button.clicked.connect(self.start_plotting)
        self._widget.graph_stop_button.clicked.connect(self.stop_plotting)
        self._widget.reset_graph_button.clicked.connect(self.reset_graph_data)

        # 데이터 저장 버튼 연결
        self._widget.save_data_button.clicked.connect(self.request_data_save)

        # Qt 타이머 추가 (UI 업데이트용)
        self.qt_timer = QTimer()
        self.qt_timer.timeout.connect(self.update_ui)
        self.qt_timer.start(100)  # 100ms 마다 UI 업데이트

        # 그래프 업데이트 타이머
        self.graph_timer = QTimer()
        self.graph_timer.timeout.connect(self.update_graphs)
        self.graph_timer.setInterval(100)  # 100ms마다 그래프 업데이트 (10Hz)

        # 초기화를 완료했음을 표시
        self._widget.status_label.setText('플러그인이 초기화되었습니다')
        self.node.get_logger().info('허리 보조 제어 플러그인이 초기화되었습니다')


    def spin_ros(self):
        """ROS 2 노드 스핀을 위한 메서드"""
        try:
            # SingleThreadedExecutor 사용
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(self.node)

            # executor 스핀
            while rclpy.ok() and not self._shutdown_flag:
                executor.spin_once(timeout_sec=0.1)  # 100ms 타임아웃으로 반응성 유지
        except Exception as e:
            if hasattr(self, 'node') and self.node:
                self.node.get_logger().error(f"ROS 스핀 스레드 오류: {str(e)}")

        finally:
            if hasattr(self, 'node') and self.node:
                self.node.get_logger().info("스핀 스레드가 종료됩니다")

    def setup_graphs(self):
        """
        그래프 위젯 초기화 및 설정
        """
        try:
            # PyQtGraph 기본 설정
            pg.setConfigOptions(antialias=True, useOpenGL=False)  # OpenGL 비활성화로 호환성 향상

            # 변위 센서 그래프 설정
            self.displacement_plot_widget = pg.PlotWidget()
            self.displacement_plot_widget.setBackground('w')
            self.displacement_plot_widget.setTitle("변위 센서 그래프", color="k", size="12pt")
            self.displacement_plot_widget.setLabel("left", "변위 값", color="k")
            self.displacement_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.displacement_plot_widget.showGrid(x=True, y=True, alpha=0.3)

            # 변위 센서 Y축 범위 설정 (0 ~ 30)
            self.displacement_plot_widget.setYRange(0, 5, padding=0.05)

            # 사용자 확대/축소 활성화
            self.displacement_plot_widget.setMouseEnabled(x=True, y=True)

            # 변위 센서 데이터 라인 생성
            self.displacement_line = self.displacement_plot_widget.plot(
                pen=pg.mkPen(color=(0, 150, 0), width=2),  # 녹색
                name="변위 센서 값",
                # symbol='o',
                # symbolSize=4,
                # symbolBrush=(0, 150, 0)
            )

            # 임계값 라인 추가
            self.threshold_line = self.displacement_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2, style=Qt.DashLine),  # 빨간색 점선
                name="임계값"
            )

            # 변위 센서 그래프 범례 추가
            displacement_legend = pg.LegendItem(offset=(60, 40))
            displacement_legend.setParentItem(self.displacement_plot_widget.graphicsItem())
            displacement_legend.addItem(self.displacement_line, "변위 센서 값")
            displacement_legend.addItem(self.threshold_line, "임계값")

            # 온도 그래프 설정
            self.temp_plot_widget = pg.PlotWidget()
            self.temp_plot_widget.setBackground('w')
            self.temp_plot_widget.setTitle("온도 그래프", color="k", size="12pt")
            self.temp_plot_widget.setLabel("left", "온도 (°C)", color="k")
            self.temp_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.temp_plot_widget.showGrid(x=True, y=True, alpha=0.3)

            # 온도 Y축 범위 설정 (20°C ~ 80°C)
            self.temp_plot_widget.setYRange(20, 100, padding=0.05)

            # 사용자 확대/축소 활성화 (자동 범위 조정은 유지)
            self.temp_plot_widget.setMouseEnabled(x=True, y=True)

            # 온도 한계선 추가 (80°C)
            limit_line = pg.InfiniteLine(
                pos=self.safety_temp_threshold,  # 80°C
                angle=0,  # 수평선
                pen=pg.mkPen(color=(255, 0, 0), width=1.5, style=Qt.DashLine),
                label=f"안전 한계: {self.safety_temp_threshold}°C",
                labelOpts={'color': (255, 0, 0), 'position': 0.95}
            )
            self.temp_plot_widget.addItem(limit_line)

            # 온도 그래프 데이터 라인 생성 (구동기 4번, 5번)
            self.temp_line_4 = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(0, 128, 255), width=2),  # 파란색 계열
                name="구동기 4 온도",
                # symbol='o',
                # symbolSize=4,
                # symbolBrush=(0, 128, 255)
            )
            self.temp_line_5 = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2),    # 빨간색
                name="구동기 5 온도",
                # symbol='o',
                # symbolSize=4,
                # symbolBrush=(255, 0, 0)
            )
            self.target_temp_line = self.temp_plot_widget.plot(
                pen=pg.mkPen(color=(0, 0, 0), width=2, style=Qt.DashLine),  # 검은색 점선
                name="목표 온도"
            )

            # 범례 추가
            self.temp_legend = pg.LegendItem(offset=(60, 40))
            self.temp_legend.setParentItem(self.temp_plot_widget.graphicsItem())
            self.temp_legend.addItem(self.temp_line_4, "구동기 4 온도")
            self.temp_legend.addItem(self.temp_line_5, "구동기 5 온도")
            self.temp_legend.addItem(self.target_temp_line, "목표 온도")

            # PWM 그래프 설정
            self.pwm_plot_widget = pg.PlotWidget()
            self.pwm_plot_widget.setBackground('w')
            self.pwm_plot_widget.setTitle("PWM 그래프", color="k", size="12pt")
            self.pwm_plot_widget.setLabel("left", "PWM 값", color="k")
            self.pwm_plot_widget.setLabel("bottom", "시간 (초)", color="k")
            self.pwm_plot_widget.showGrid(x=True, y=True, alpha=0.3)

            # PWM Y축 범위 설정 (0-100)
            self.pwm_plot_widget.setYRange(0, 100, padding=0.05)

            # 사용자 확대/축소 활성화
            self.pwm_plot_widget.setMouseEnabled(x=True, y=True)

            # PWM 그래프 데이터 라인 생성 (구동기 4번, 5번)
            self.pwm_line_4 = self.pwm_plot_widget.plot(
                pen=pg.mkPen(color=(0, 128, 255), width=2),  # 파란색 계열
                name="구동기 4 PWM",
                # symbol='o',
                # symbolSize=4,
                # symbolBrush=(0, 128, 255)
            )
            self.pwm_line_5 = self.pwm_plot_widget.plot(
                pen=pg.mkPen(color=(255, 0, 0), width=2),    # 빨간색
                name="구동기 5 PWM",
                # symbol='o',
                # symbolSize=4,
                # symbolBrush=(255, 0, 0)
            )

            # PWM 범례 추가
            pwm_legend = pg.LegendItem(offset=(60, 40))
            pwm_legend.setParentItem(self.pwm_plot_widget.graphicsItem())
            pwm_legend.addItem(self.pwm_line_4, "구동기 4 PWM")
            pwm_legend.addItem(self.pwm_line_5, "구동기 5 PWM")

            # X축 연결 (세 그래프의 시간축을 동기화)
            self.displacement_plot_widget.setXLink(self.temp_plot_widget)
            self.temp_plot_widget.setXLink(self.pwm_plot_widget)

            # 기존 레이아웃 제거 (이미 레이아웃이 있을 경우)
            for frame_name in ['displacement_graph_frame', 'temp_graph_frame', 'pwm_graph_frame']:
                frame = getattr(self._widget, frame_name)
                if frame.layout() is not None:
                    old_layout = frame.layout()
                    while old_layout.count():
                        item = old_layout.takeAt(0)
                        widget = item.widget()
                        if widget is not None:
                            widget.deleteLater()
                    # Qt는 레이아웃을 직접 삭제하지 않으므로 빈 위젯으로 대체
                    QWidget().setLayout(old_layout)

            # 그래프 프레임에 위젯 추가
            displacement_layout = QVBoxLayout()
            displacement_layout.addWidget(self.displacement_plot_widget)
            self._widget.displacement_graph_frame.setLayout(displacement_layout)

            temp_layout = QVBoxLayout()
            temp_layout.addWidget(self.temp_plot_widget)
            self._widget.temp_graph_frame.setLayout(temp_layout)

            pwm_layout = QVBoxLayout()
            pwm_layout.addWidget(self.pwm_plot_widget)
            self._widget.pwm_graph_frame.setLayout(pwm_layout)

            self.node.get_logger().info("그래프 초기화 완료")
        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 초기화 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())
            # UI에 오류 메시지 표시
            error_layout = QVBoxLayout()
            error_label = QLabel(f"그래프 초기화 오류: {str(e)}")
            error_label.setStyleSheet("color: red; font-weight: bold;")
            error_layout.addWidget(error_label)
            self._widget.temp_graph_frame.setLayout(error_layout)
            self._widget.pwm_graph_frame.setLayout(QVBoxLayout())
            self._widget.displacement_graph_frame.setLayout(QVBoxLayout())

    def collect_data_for_graph(self):
        """
        그래프용 데이터를 주기적으로 수집하는 함수
        """
        # 그래프 활성화 상태가 아니면 데이터 수집 생략
        if not self.is_plotting or not hasattr(self, 'start_time') or self.start_time is None:
            return

        try:
            current_time = datetime.datetime.now()
            elapsed_time = (current_time - self.start_time).total_seconds()

            # 데이터 락 획득 후 일괄 처리
            with self.data_lock:
                # 현재 시점의 모든 센서 데이터를 한번에 기록
                self.time_data.append(elapsed_time)

                # 변위 센서값과 임계값
                self.displacement_data.append(self.displacement)
                self.threshold_data.append(self.applied_angle_threshold)

                # 온도 데이터 (4번, 5번 구동기)
                temp_4 = self.temperatures[4] if len(self.temperatures) > 4 else 0.0
                temp_5 = self.temperatures[5] if len(self.temperatures) > 5 else 0.0
                self.temp_data_4.append(temp_4)
                self.temp_data_5.append(temp_5)

                # 목표 온도
                self.target_temp_data.append(self.applied_target_temp)

                # PWM 값
                pwm_4 = self.pwm_values[4] if len(self.pwm_values) > 4 else 0
                pwm_5 = self.pwm_values[5] if len(self.pwm_values) > 5 else 0
                self.pwm_data_4.append(pwm_4)
                self.pwm_data_5.append(pwm_5)

                # 버퍼 크기 제한 (5분 = 300초 기준)
                max_age = 300.0  # 5분
                while self.time_data and (elapsed_time - self.time_data[0]) > max_age:
                    self.time_data.pop(0)
                    self.displacement_data.pop(0)
                    self.threshold_data.pop(0)
                    self.temp_data_4.pop(0)
                    self.temp_data_5.pop(0)
                    self.target_temp_data.pop(0)
                    self.pwm_data_4.pop(0)
                    self.pwm_data_5.pop(0)

                # 주기적으로 로그 출력 (1000개 데이터 포인트마다)
                if len(self.time_data) % 1000 == 0:
                    self.node.get_logger().info(f"데이터 개수: {len(self.time_data)}, 시간 범위: {elapsed_time - self.time_data[0]:.2f}초")

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 데이터 수집 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

    def update_ui(self):
        """
        UI를 업데이트하는 함수 (Qt 타이머에서 호출)
        """
        if not hasattr(self, '_widget') or self._widget is None:
            return

        try:
            # 변위 센서 값 표시
            self._widget.displacement_label.setText(f"{self.displacement:.2f}")

            # 구동기 4, 5번 온도와 PWM 값 업데이트
            if len(self.temperatures) > 4:
                self._widget.temp_label_4.setText(f"{self.temperatures[4]:.2f}°C")
            if len(self.temperatures) > 5:
                self._widget.temp_label_5.setText(f"{self.temperatures[5]:.2f}°C")

            if len(self.pwm_values) > 4:
                self._widget.pwm_label_4.setText(f"PWM: {self.pwm_values[4]}")
            if len(self.pwm_values) > 5:
                self._widget.pwm_label_5.setText(f"PWM: {self.pwm_values[5]}")

            # 의도 상태 표시
            intention_text = "있음" if self.current_intention else "없음"
            self._widget.intention_status_label.setText(f"현재 의도: {intention_text}")
            self._widget.intention_button.setText("의도 OFF" if self.current_intention else "의도 ON")

            # 허리 보조 상태 표시
            if self.is_active:
                self._widget.activation_status_label.setText("활성화됨")
                self._widget.activation_status_label.setStyleSheet("color: green; font-weight: bold;")
            else:
                self._widget.activation_status_label.setText("비활성화됨")
                self._widget.activation_status_label.setStyleSheet("color: gray;")

            # 비상 정지 상태 표시
            if self.is_emergency_stop:
                self._widget.status_label.setText("비상 정지 상태")
                self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")

                # 비상 정지 중에는 버튼 상태 조정
                self._widget.apply_button.setEnabled(False)
                self._widget.emergency_stop_button.setText("비상 정지 해제")
                # 설정 그룹 비활성화
                self._widget.settings_group.setEnabled(False)
                self._widget.intention_group.setEnabled(False)
            else:
                # 비상 정지가 아닌 경우 버튼 상태 복원
                self._widget.apply_button.setEnabled(True)
                self._widget.emergency_stop_button.setText("비상 정지")
                # 설정 그룹 활성화
                self._widget.settings_group.setEnabled(True)
                self._widget.intention_group.setEnabled(True)

            # 그래프 버튼 상태 업데이트
            self._widget.graph_start_button.setEnabled(not self.is_plotting)
            self._widget.graph_stop_button.setEnabled(self.is_plotting)

            # 로깅 상태 업데이트
            if self.recording_process is not None:
                # 이미 로깅 상태 문자열이 있는지 확인하고, 없으면 기본 메시지 표시
                status_text = self._widget.log_status_label.text()
                self._widget.save_data_button.setText("로깅 중지")
                if "로깅 상태: 활성화" not in status_text:
                    self._widget.log_status_label.setText("로깅 상태: 활성화")
            else:
                self._widget.save_data_button.setText("데이터 저장")

        except RuntimeError as e:
            if "deleted" in str(e):
                # 위젯이 삭제된 경우 타이머 중지
                if hasattr(self, 'qt_timer'):
                    self.qt_timer.stop()

    def update_graphs(self):
        """
        그래프 데이터 업데이트 (그래프 타이머에서 호출)
        """
        if not self.is_plotting:
            return

        try:
            # 데이터가 있을 경우에만 그래프 업데이트
            if all(hasattr(self, attr) for attr in ['displacement_line', 'threshold_line', 'temp_line_4', 'temp_line_5', 'pwm_line_4', 'pwm_line_5']):
                # 락 획득 후 데이터 복사 (스레드 안전)
                with self.data_lock:
                    if not self.time_data or len(self.time_data) < 2:
                        return  # 데이터가 없거나 불충분하면 업데이트 중단

                    # 데이터 안전하게 복사
                    time_data = list(self.time_data)
                    displacement_data = list(self.displacement_data)
                    threshold_data = list(self.threshold_data)
                    temp_data_4 = list(self.temp_data_4)
                    temp_data_5 = list(self.temp_data_5)
                    target_temp_data = list(self.target_temp_data)
                    pwm_data_4 = list(self.pwm_data_4)
                    pwm_data_5 = list(self.pwm_data_5)

                # 일관성 검사 (추가 보호)
                min_length = min(len(time_data), len(displacement_data), len(threshold_data),
                                len(temp_data_4), len(temp_data_5),
                                len(target_temp_data), len(pwm_data_4), len(pwm_data_5))
                if min_length < 2:
                    return  # 충분한 데이터가 없으면 업데이트 중단

                # 모든 배열을 같은 길이로 자름
                time_data = time_data[:min_length]
                displacement_data = displacement_data[:min_length]
                threshold_data = threshold_data[:min_length]
                temp_data_4 = temp_data_4[:min_length]
                temp_data_5 = temp_data_5[:min_length]
                target_temp_data = target_temp_data[:min_length]
                pwm_data_4 = pwm_data_4[:min_length]
                pwm_data_5 = pwm_data_5[:min_length]

                # 시간 창 계산 (지정된 시간 창만 표시)
                window_size = 60.0  # 60초 창
                latest_time = time_data[-1]
                start_time = max(0, latest_time - window_size)

                # 시간 범위로 필터링
                visible_data = [(i, t, d, th, v1, v2, v3, v4, v5) for i, (t, d, th, v1, v2, v3, v4, v5) in
                                enumerate(zip(time_data, displacement_data, threshold_data,
                                              temp_data_4, temp_data_5,
                                              target_temp_data, pwm_data_4, pwm_data_5))
                                if t >= start_time]

                if not visible_data:
                    return

                # 인덱스, 시간, 변위, 임계값, 온도들, PWM들로 분리
                indices, display_time, display_displacement, display_threshold, display_temp_4, display_temp_5, display_target, display_pwm_4, display_pwm_5 = zip(*visible_data)

                # 다운샘플링 로직 - 최대 400개 포인트로 제한
                target_points = 400

                if len(display_time) > target_points:
                    # 다운샘플링 인덱스 계산
                    indices = list(indices)  # 튜플을 리스트로 변환

                    # 다운샘플링 방법: 시간 기준 균등 분할
                    if len(indices) >= 2:
                        # 첫 포인트와 마지막 포인트
                        sampled_indices = [indices[0]]

                        # 중간 포인트들 (균등 간격)
                        step = (len(indices) - 2) / (target_points - 2)
                        for i in range(1, target_points - 1):
                            idx = indices[min(int(i * step) + 1, len(indices) - 1)]
                            sampled_indices.append(idx)

                        # 마지막 포인트 추가
                        sampled_indices.append(indices[-1])

                        # 중복 제거 및 정렬
                        sampled_indices = sorted(set(sampled_indices))

                        # 샘플링된 데이터로 새로운 데이터셋 생성
                        display_time = [time_data[i] for i in sampled_indices]
                        display_displacement = [displacement_data[i] for i in sampled_indices]
                        display_threshold = [threshold_data[i] for i in sampled_indices]
                        display_temp_4 = [temp_data_4[i] for i in sampled_indices]
                        display_temp_5 = [temp_data_5[i] for i in sampled_indices]
                        display_target = [target_temp_data[i] for i in sampled_indices]
                        display_pwm_4 = [pwm_data_4[i] for i in sampled_indices]
                        display_pwm_5 = [pwm_data_5[i] for i in sampled_indices]

                # 그래프 업데이트
                self.displacement_line.setData(display_time, display_displacement)
                self.threshold_line.setData(display_time, display_threshold)
                self.temp_line_4.setData(display_time, display_temp_4)
                self.temp_line_5.setData(display_time, display_temp_5)
                self.target_temp_line.setData(display_time, display_target)
                self.pwm_line_4.setData(display_time, display_pwm_4)
                self.pwm_line_5.setData(display_time, display_pwm_5)

                # X축 범위 고정 (60초 창)
                self.displacement_plot_widget.setXRange(start_time, latest_time, padding=0.02)
                self.temp_plot_widget.setXRange(start_time, latest_time, padding=0.02)
                self.pwm_plot_widget.setXRange(start_time, latest_time, padding=0.02)

                # 자동 범위 조정 비활성화
                self.displacement_plot_widget.getViewBox().setAutoVisible(y=True, x=False)
                self.temp_plot_widget.getViewBox().setAutoVisible(y=True, x=False)
                self.pwm_plot_widget.getViewBox().setAutoVisible(y=True, x=False)

                # 상태 업데이트
                if hasattr(self, '_widget'):
                    self._widget.status_label.setText(f"데이터 플로팅 중: {len(display_time)}포인트 ({len(time_data)}개 중)")

        except Exception as e:
            import traceback
            self.node.get_logger().error(f"그래프 업데이트 오류: {str(e)}")
            self.node.get_logger().error(traceback.format_exc())

    def temp_callback(self, msg):
        """
        온도 데이터 수신 콜백 - 데이터 저장만 수행
        """
        try:
            # 단순히 현재 온도값만 저장
            self.temperatures = list(msg.temperature)

            # 로그 출력 (디버그 레벨로만)
            self.node.get_logger().debug(f'온도 데이터 수신: 4번={self.temperatures[4] if len(self.temperatures) > 4 else 0}, '
                                        f'5번={self.temperatures[5] if len(self.temperatures) > 5 else 0}')
        except Exception as e:
            self.node.get_logger().error(f'온도 데이터 처리 오류: {str(e)}')

    def pwm_callback(self, msg):
        """
        현재 PWM 상태 수신 콜백 - 데이터 저장만 수행
        """
        try:
            # 단순히 현재 PWM 값만 저장
            self.pwm_values = list(msg.pwm)
            #self.node.get_logger().debug(f'PWM 데이터 수신: 4번={self.pwm_values[4] if len(self.pwm_values) > 4 else 0}, 'f'5번={self.pwm_values[5] if len(self.pwm_values) > 5 else 0}')
        except Exception as e:
            self.node.get_logger().error(f'PWM 데이터 처리 오류: {str(e)}')

    def displacement_callback(self, msg):
        """
        변위 센서 데이터 수신 콜백 - 데이터 저장만 수행
        """
        try:
            # 단순히 현재 변위값만 저장
            if len(msg.displacement) > 0:
                self.displacement = msg.displacement[0]
                #self.node.get_logger().debug(f'변위 센서 데이터 수신: {self.displacement:.3f}')
        except Exception as e:
            self.node.get_logger().error(f'변위 데이터 처리 오류: {str(e)}')

    def toggle_intention(self):
        """
        의도 ON/OFF 토글 버튼 클릭 핸들러
        """
        try:
            # 현재 의도 상태 반전
            new_intention = 0 if self.current_intention else 3  # 0: 없음, 3: 굽히기(기본 의도)

            # 의도 메시지 생성 및 발행
            msg = BackIntension()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.intention_id = new_intention

            self.intention_pub.publish(msg)
            self.current_intention = new_intention

            # 로그 출력
            intention_text = "ON" if new_intention else "OFF"
            self.node.get_logger().info(f'의도 {intention_text} 설정 (ID: {new_intention})')
        except Exception as e:
            self.node.get_logger().error(f'의도 설정 오류: {str(e)}')

    def apply_settings(self):
        """현재 UI 설정에 따라 제어 명령 발행"""
        if self.is_emergency_stop:
            self.node.get_logger().warn('비상 정지 중입니다. 제어 명령이 무시됩니다.')
            return  # 비상 정지 중에는 명령 무시

        try:
            # 목표 온도 설정값 발행
            target_temp = self._widget.target_temp_spin.value()
            self.publish_target_temperature(target_temp)

            # 적용된 값 업데이트
            self.applied_target_temp = target_temp

            # 임계값 업데이트
            threshold_value = self._widget.angle_threshold_spin.value()
            self.applied_angle_threshold = threshold_value
            self.applied_angle_threshold = threshold_value  # 내부 작업용 변수도 함께 업데이트

            self._widget.status_label.setText(f'목표 온도 {target_temp:.2f}°C, 임계값 {threshold_value:.2f}로 설정')
            self.node.get_logger().info(f'설정 적용: 목표 온도={target_temp:.2f}°C, 임계값={threshold_value:.2f}')
        except Exception as e:
            self.node.get_logger().error(f'설정 적용 오류: {str(e)}')
            self._widget.status_label.setText(f'설정 적용 오류: {str(e)}')

    def publish_target_temperature(self, target_temp):
        """
        목표 온도 설정 메시지 발행
        """
        try:
            # 단일 값을 Float64 메시지로 발행
            msg = std_msgs.msg.Float64()
            msg.data = float(target_temp)
            self.target_temp_pub.publish(msg)
            self.node.get_logger().info(f'목표 온도 설정 메시지 발행: {target_temp:.2f}°C')
        except Exception as e:
            self.node.get_logger().error(f'목표 온도 발행 오류: {str(e)}')

    def start_plotting(self):
        """
        그래프 플로팅 시작
        """
        if self.is_plotting:
            return

        # 그래프 데이터 초기화
        self.reset_graph_data()

        # 플로팅 상태 활성화
        self.is_plotting = True
        self.start_time = datetime.datetime.now()

        # 그래프 타이머 시작
        self.graph_timer.start()

        # UI 상태 업데이트
        self._widget.graph_start_button.setEnabled(False)
        self._widget.graph_stop_button.setEnabled(True)
        self._widget.status_label.setText("그래프 플로팅이 시작되었습니다")
        self.node.get_logger().info("그래프 플로팅이 시작되었습니다")

        # 초기 데이터 추가 - 단순화된 버전
        with self.data_lock:
            self.time_data = [0.0]
            self.displacement_data = [self.displacement]
            self.threshold_data = [self.applied_angle_threshold]

            # 현재 저장된 센서값으로 초기화
            temp_4 = self.temperatures[4] if len(self.temperatures) > 4 else 0.0
            temp_5 = self.temperatures[5] if len(self.temperatures) > 5 else 0.0
            self.temp_data_4 = [temp_4]
            self.temp_data_5 = [temp_5]

            # 목표 온도 초기값
            self.target_temp_data = [self.applied_target_temp]

            # PWM 초기값
            pwm_4 = self.pwm_values[4] if len(self.pwm_values) > 4 else 0
            pwm_5 = self.pwm_values[5] if len(self.pwm_values) > 5 else 0
            self.pwm_data_4 = [pwm_4]
            self.pwm_data_5 = [pwm_5]

    def stop_plotting(self):
        """
        그래프 플로팅 중지
        """
        if not self.is_plotting:
            return

        # 플로팅 상태 비활성화
        self.is_plotting = False

        # 그래프 타이머 중지
        self.graph_timer.stop()

        # UI 상태 업데이트
        self._widget.graph_start_button.setEnabled(True)
        self._widget.graph_stop_button.setEnabled(False)
        self._widget.status_label.setText("그래프 플로팅이 중지되었습니다")
        self.node.get_logger().info("그래프 플로팅이 중지되었습니다")

    def reset_graph_data(self):
        """
        그래프 데이터 초기화
        """
        # 데이터 배열 초기화
        self.time_data = []
        self.displacement_data = []
        self.threshold_data = []
        self.temp_data_4 = []
        self.temp_data_5 = []
        self.target_temp_data = []
        self.pwm_data_4 = []
        self.pwm_data_5 = []

        # 시작 시간 재설정
        self.start_time = datetime.datetime.now()

        # 그래프 초기화
        if all(hasattr(self, attr) for attr in ['displacement_line', 'threshold_line', 'temp_line_4', 'temp_line_5', 'target_temp_line', 'pwm_line_4', 'pwm_line_5']):
            self.displacement_line.setData([], [])
            self.threshold_line.setData([], [])
            self.temp_line_4.setData([], [])
            self.temp_line_5.setData([], [])
            self.target_temp_line.setData([], [])
            self.pwm_line_4.setData([], [])
            self.pwm_line_5.setData([], [])

            # 그래프 범위 초기화
            if all(hasattr(self, attr) for attr in ['displacement_plot_widget', 'temp_plot_widget', 'pwm_plot_widget']):
                self.displacement_plot_widget.setXRange(0, 300)
                self.temp_plot_widget.setXRange(0, 300)
                self.pwm_plot_widget.setXRange(0, 300)

        self._widget.status_label.setText("그래프 데이터가 초기화되었습니다")
        self.node.get_logger().info("그래프 데이터가 초기화되었습니다")

    def check_state(self):
        """
        안전 검사 및 상태 확인을 위한 타이머 콜백
        """
        # 온도 안전성 검사
        self.check_temperature_safety()

        # 상태 확인
        self.check_activation_status()

    def check_temperature_safety(self):
        """
        온도가 안전 범위 내에 있는지 확인하고 필요시 비상 정지 활성화
        """
        # 구동기 4번과 5번의 온도 확인
        temp_4 = self.temperatures[4] if len(self.temperatures) > 4 else 0.0
        temp_5 = self.temperatures[5] if len(self.temperatures) > 5 else 0.0

        # 어느 하나라도 임계값을 초과하면 비상 정지
        if (temp_4 >= self.safety_temp_threshold or temp_5 >= self.safety_temp_threshold) and not self.is_emergency_stop:
            self.node.get_logger().error(
                f'온도 임계값 초과! 구동기 4번: {temp_4:.2f}°C, 구동기 5번: {temp_5:.2f}°C, 임계값: {self.safety_temp_threshold:.2f}°C')

            # 비상 정지 활성화
            self.is_emergency_stop = True
            self.set_emergency_stop(True)

            # UI 업데이트 (비동기적으로 실행되므로 메인 스레드에서 처리 필요)
            # Qt의 스레드 안전성을 위해 QTimer.singleShot 사용
            QTimer.singleShot(0, lambda: self._show_temperature_warning(temp_4, temp_5))

    def check_activation_status(self):
        """
        허리 보조 시스템 활성화 상태 확인
        """
        # 활성화 조건 검사
        new_active_state = self.current_intention != 0 and self.displacement > self.applied_angle_threshold

        # 상태가 변경되었을 때만 처리
        if new_active_state != self.is_active:
            self.is_active = new_active_state
            mode_str = "자동 온도제어" if self.is_active else "수동 PWM 제어"
            self.node.get_logger().info(f"활성화 상태 변경: {self.is_active}")

            if self.control_mode_client.service_is_ready():
                # 서비스 요청 생성
                request = SetControlMode.Request()
                request.auto_mode = self.is_active  # 활성화 상태에 따라 자동/수동 모드 설정
                request.actuator_id = -1  # -1은 모든 구동기를 의미

                # 비동기 서비스 호출
                self.node.get_logger().info(f"서비스 호출: set_control_mode, auto_mode={self.is_active}")
                future = self.control_mode_client.call_async(request)
                future.add_done_callback(self.handle_control_mode_response)
            else:
                self.node.get_logger().warn("제어 모드 서비스가 준비되지 않았습니다. 상태 변경이 적용되지 않습니다.")

    def handle_control_mode_response(self, future):
        """제어 모드 서비스 응답 처리"""
        try:
            response = future.result()
            if response.success:
                mode_str = "자동 온도 제어" if self.is_active else "수동 PWM 제어"
                self.node.get_logger().info(f"제어 모드 변경 성공: {mode_str}")
                self._widget.status_label.setText(f"제어 모드가 {mode_str}로 변경되었습니다")
            else:
                self.node.get_logger().error(f"제어 모드 변경 실패: {response.message}")
                self._widget.status_label.setText(f"제어 모드 변경 실패: {response.message}")
        except Exception as e:
            self.node.get_logger().error(f"제어 모드 서비스 호출 오류: {str(e)}")

    def _show_temperature_warning(self, temp_4, temp_5):
        """
        온도 경고 메시지 표시 (메인 스레드에서 실행)
        """
        warning_msg = QMessageBox()
        warning_msg.setIcon(QMessageBox.Warning)
        warning_msg.setWindowTitle("온도 경고")
        warning_msg.setText("구동기 온도가 안전 임계값을 초과했습니다!")
        warning_msg.setInformativeText(f"구동기 4번: {temp_4:.2f}°C\n"
                                       f"구동기 5번: {temp_5:.2f}°C\n"
                                       f"임계값: {self.safety_temp_threshold:.2f}°C\n\n"
                                       f"비상 정지가 활성화되었습니다.")
        warning_msg.setStandardButtons(QMessageBox.Ok)
        warning_msg.exec_()

    def emergency_stop(self):
        """비상 정지 버튼 이벤트 처리 - 서비스 호출 방식으로 변경"""
        # 현재 상태의 반대로 설정
        activate = not self.is_emergency_stop

        self._widget.status_label.setText(f"비상 정지 {'활성화' if activate else '해제'} 요청 중...")

        # 서비스 호출로 비상 정지 요청
        self.set_emergency_stop(activate)

    def set_emergency_stop(self, activate):
        """비상 정지 설정 서비스 호출"""
        if not self.service_availability['set_emergency_stop']:
            self.node.get_logger().error('비상 정지 서비스가 준비되지 않았습니다')
            # 사용자에게 알림
            self._widget.status_label.setText('비상 정지 서비스가 준비되지 않았습니다')
            self._widget.status_label.setStyleSheet("color: red;")
            return

        request = EmergencyStop.Request()
        request.activate = activate

        # 비동기 서비스 호출
        future = self.emergency_client.call_async(request)
        future.add_done_callback(lambda f: self.handle_emergency_response(f, activate))

    def handle_emergency_response(self, future, activate):
        """비상 정지 서비스 응답 처리"""
        try:
            response = future.result()
            if response.success:
                self.is_emergency_stop = activate
                action_str = "활성화" if activate else "해제"
                self._widget.status_label.setText(f'비상 정지가 {action_str}되었습니다')

                if activate:
                    # 비상 정지 활성화 시
                    self._widget.status_label.setStyleSheet("color: red; font-weight: bold;")
                    # 의도 ID를 0으로 설정 (동작 중지)
                    self.current_intention = 0
                    self.toggle_intention()  # 의도 OFF 상태로 설정
                    self.node.get_logger().error(f'비상 정지가 활성화되었습니다: {response.message}')
                else:
                    # 비상 정지 해제 시
                    self._widget.status_label.setStyleSheet("")
                    self.node.get_logger().info(f'비상 정지가 해제되었습니다: {response.message}')
            else:
                action_str = "활성화" if activate else "해제"
                self._widget.status_label.setText(f'비상 정지 {action_str} 실패: {response.message}')
                self.node.get_logger().error(f'비상 정지 {action_str} 실패: {response.message}')
        except Exception as e:
            self._widget.status_label.setText(f'비상 정지 요청 오류: {str(e)}')
            self.node.get_logger().error(f'비상 정지 요청 오류: {str(e)}')

    def request_data_save(self):
        """
        ROS2 bag 기록 시작/중지
        """
        try:
            # 이미 녹화 중인지 확인
            if self.recording_process is not None:
                # 녹화 중지
                try:
                    # 프로세스 그룹 전체를 종료하여 자식 프로세스도 함께 종료
                    os.killpg(os.getpgid(self.recording_process.pid), signal.SIGTERM)
                except:
                    # 이미 종료된 경우 등의 예외 처리
                    pass

                self.recording_process = None

                self._widget.status_label.setText("녹화가 중지되었습니다")
                self._widget.save_data_button.setText("데이터 저장")
                self.node.get_logger().info("ROS2 bag 녹화가 중지되었습니다")

                # 로깅 상태 메시지 업데이트
                self._widget.log_status_label.setText("로깅 상태: 대기 중")

            else:
                # 녹화 시작
                # 파일명에 날짜/시간 추가
                base_name = 'wearable_robot '
                # if not base_name:
                #     base_name = "waist_robot"

                timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
                output_dir = os.path.expanduser(f"~/ros2_bags/{base_name}_{timestamp}")

                # 디렉토리가 없으면 생성
                os.makedirs(os.path.dirname(output_dir), exist_ok=True)

                # 저장할 토픽 목록 (허리 보조 로봇 관련 토픽 추가)
                topics = [
                    "/temperature_data",
                    "/displacement_data",
                    "/imu_data",
                    "/fan_state",
                    "/pwm_state",
                    "/intention_data",
                    "/target_temperature",
                    "/target_temperature_setting"
                ]

                # ROS2 bag 녹화 명령 구성
                cmd = ["ros2", "bag", "record", "-o", output_dir]
                cmd.extend(topics)

                # 새 프로세스 그룹으로 실행 (나중에 종료하기 쉽게)
                self.recording_process = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,  # 출력 리다이렉션으로 오버헤드 감소
                    stderr=subprocess.DEVNULL
                )

                self._widget.status_label.setText(f"녹화가 시작되었습니다: {os.path.basename(output_dir)}")
                self._widget.save_data_button.setText("녹화 중지")
                self.node.get_logger().info(f"ROS2 bag 녹화가 시작되었습니다: {output_dir}")

                # 로깅 상태 메시지 업데이트
                self._widget.log_status_label.setText(f"로깅 상태: 활성화 - {os.path.basename(output_dir)}")

                # 그래프 플로팅도 자동으로 시작
                if not self.is_plotting:
                    self.start_plotting()

        except Exception as e:
            error_msg = f"ROS2 bag 녹화 오류: {str(e)}"
            self._widget.status_label.setText(error_msg)
            self.node.get_logger().error(error_msg)

            # 오류 메시지 표시
            QMessageBox.critical(self._widget, "녹화 오류", f"ROS2 bag 녹화 중 오류가 발생했습니다.\n{str(e)}")

    def check_service_availability(self):
        """서비스 가용성 확인 및 UI 업데이트"""
        self.service_availability['set_control_mode'] = self.control_mode_client.service_is_ready()
        self.service_availability['set_pi_parameters'] = self.pi_params_client.service_is_ready()
        self.service_availability['set_emergency_stop'] = self.emergency_client.service_is_ready()

        if not all(self.service_availability.values()):
            # 사용 불가능한 서비스가 있는 경우 더 명확한 경고
            unavailable = [name for name, available in self.service_availability.items() if not available]
            self._widget.service_status_label.setText(f"서비스 대기 중: {', '.join(unavailable)}")
            self._widget.service_status_label.setStyleSheet("color: orange;")
        else:
            self._widget.service_status_label.setText("모든 서비스 연결됨")
            self._widget.service_status_label.setStyleSheet("color: green;")

        if all(self.service_availability.values()) and not hasattr(self, 'initial_state_sent'):
            # 초기 제어 모드 설정 (기본값은 활성화 상태에 따라)
            request = SetControlMode.Request()
            request.auto_mode = self.is_active  # 현재 활성화 상태에 맞게 설정
            request.actuator_id = -1  # 모든 구동기

            # 비동기 서비스 호출
            future = self.control_mode_client.call_async(request)
            future.add_done_callback(self.handle_control_mode_response)

            # 초기 상태 설정 완료 표시
            self.initial_state_sent = True
            self.node.get_logger().info("초기 제어 모드가 설정되었습니다")

        # 로그로 서비스 가용성 출력
        self.node.get_logger().debug(
            f"서비스 가용성: 제어모드={self.service_availability['set_control_mode']}, "
            f"PI파라미터={self.service_availability['set_pi_parameters']}, "
            f"비상정지={self.service_availability['set_emergency_stop']}")

    def shutdown_plugin(self):
        """플러그인이 종료될 때 호출되는 메서드"""

        self._shutdown_flag = True

        # Qt 타이머 중지
        if hasattr(self, 'qt_timer') and self.qt_timer.isActive():
            self.qt_timer.stop()

        # 그래프 타이머 중지
        if hasattr(self, 'graph_timer') and self.graph_timer.isActive():
            self.graph_timer.stop()

        # ROS2 bag 녹화 프로세스 종료
        if hasattr(self, 'recording_process') and self.recording_process is not None:
            try:
                os.killpg(os.getpgid(self.recording_process.pid), signal.SIGTERM)
                self.node.get_logger().info("ROS2 bag 녹화가 종료되었습니다")
            except:
                pass

        # 스핀 스레드 종료 대기
        if hasattr(self, 'spin_thread') and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)  # 최대 1초 대기

        # ROS 노드 정리
        if hasattr(self, 'node') and self.node:
            self.node.get_logger().info("플러그인 종료 중...")
            self.node.destroy_node()

        try:
            if self.is_emergency_stop and self.emergency_client.service_is_ready():
                request = EmergencyStop.Request()
                request.activate = False
                # 동기 호출 사용 (종료 중이므로)
                self.emergency_client.call(request)
                self.node.get_logger().info("종료 전 비상 정지 해제")
        except Exception as e:
            self.node.get_logger().error(f"종료 시 비상 정지 해제 실패: {str(e)}")

        print("[INFO]: 플러그인 자원이 정상적으로 해제되었습니다")

def main(args=None):
    """
    rqt 플러그인 메인 함수 (독립 실행 모드에서 사용)
    """
    # 독립 실행 모드일 때만 rclpy.init을 호출합니다
    # rqt 내에서 실행될 때는 이미 초기화되어 있으므로 호출하지 않습니다

    from rqt_gui.main import Main
    main = Main()
    sys.exit(main.main(sys.argv, standalone='wearable_robot_rqt_plugins.waist_control_plugin.WaistControlPlugin'))

if __name__ == '__main__':
    import sys

    # 독립 실행 모드일 때는 rclpy를 직접 초기화합니다
    rclpy.init(args=sys.argv)
    main()
