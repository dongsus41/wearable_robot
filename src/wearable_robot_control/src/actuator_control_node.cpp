#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <wearable_robot_interfaces/srv/set_control_mode.hpp>
#include <wearable_robot_interfaces/srv/set_control_params.hpp>
#include <wearable_robot_interfaces/srv/emergency_stop.hpp>
#include <string>
#include <vector>
#include <mutex>

class ActuatorControlNode : public rclcpp::Node
{
public:
    ActuatorControlNode() : Node("actuator_control_node")
    {
        // 온도 데이터 구독
        temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10,
            std::bind(&ActuatorControlNode::temperature_callback, this, std::placeholders::_1));

        // 명령 발행
        pwm_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10);

        // auto mode: 목표 온도 구독
        target_temp_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::TemperatureData>(
            "target_temperature", 10,
            std::bind(&ActuatorControlNode::target_temp_callback, this, std::placeholders::_1));

        // pwm mode: 직접 PWM 값 구독
        direct_pwm_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "direct_pwm_command", 10,
            std::bind(&ActuatorControlNode::direct_pwm_callback, this, std::placeholders::_1));

        // 제어 모드 변경 서비스
        control_mode_service_ = this->create_service<wearable_robot_interfaces::srv::SetControlMode>(
            "set_control_mode",
            std::bind(&ActuatorControlNode::handle_control_mode, this,
                      std::placeholders::_1, std::placeholders::_2));

        // PI 파라미터 설정 서비스
        pi_param_service_ = this->create_service<wearable_robot_interfaces::srv::SetControlParams>(
            "set_pi_parameters",
            std::bind(&ActuatorControlNode::handle_pi_parameters, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 비상 정지 서비스
        emergency_service_ = this->create_service<wearable_robot_interfaces::srv::EmergencyStop>(
            "set_emergency_stop",
            std::bind(&ActuatorControlNode::handle_emergency_stop, this,
                      std::placeholders::_1, std::placeholders::_2));


        // PI 제어 파라미터 및 목표 온도 선언
        this->declare_parameter("target_temperature", 50.0);  // 기본값 50도
        this->declare_parameter("kp", 2.0);                  // 비례 게인
        this->declare_parameter("ki", 0.1);                  // 적분 게인
        this->declare_parameter("max_pwm", 100.0);           // 최대 PWM
        this->declare_parameter("min_pwm", 0.0);             // 최소 PWM
        this->declare_parameter("active_actuator", 5);       // 제어할 구동기 번호
        this->declare_parameter("safety_threshold", 80.0);   // 안전 온도 임계값 (섭씨)
        this->declare_parameter("control_frequency", 250.0); // 제어 주파수 (Hz)

        // 파라미터 변경 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&ActuatorControlNode::parameter_callback, this, std::placeholders::_1));


        // 제어 타이머 (250Hz로 제어)
        double control_frequency = this->get_parameter("control_frequency").as_double();
        int period_ms = static_cast<int>(1000.0 / control_frequency);

        // 제어 타이머 생성 (control callback 함수 250Hz로 호출)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&ActuatorControlNode::control_callback, this));

        // 내부 변수 초기화
        current_temp_ = 0.0;
        previous_error_ = 0.0;
        integral_ = 0.0;
        pwm_output_ = 0;
        is_emergency_stop_ = false;
        is_temperature_safe_ = true;
        use_direct_pwm_ = true;

        // PWM 값 배열 초기화
        current_pwm_values_.resize(6, 0);

        // 액티브 액추에이터 인덱스 초기화
        active_actuator_idx_ = this->get_parameter("active_actuator").as_int();

        // 초기화 후 메시지 출력
        RCLCPP_INFO(this->get_logger(), "액추에이터 제어 노드가 시작되었습니다");
        RCLCPP_INFO(this->get_logger(), "액티브 액추에이터: %d번 (actuator 0 ~ 5)",active_actuator_idx_);
        RCLCPP_INFO(this->get_logger(), "안전 온도 임계값: %.1f°C", this->get_parameter("safety_threshold").as_double());
        RCLCPP_INFO(this->get_logger(), "제어 주파수: %.1fHz", control_frequency);
        RCLCPP_INFO(this->get_logger(), "초기 제어 모드: %s", use_direct_pwm_ ? "수동 제어" : "자동 제어");

        // 초기 PWM 상태 발행
        publish_pwm_command();
    }

private:
    // 온도 데이터 수신 콜백
    void temperature_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 온도 센서 데이터 확인
        if (msg->temperature.size() > static_cast<size_t>(active_actuator_idx_)) {
            current_temp_ = msg->temperature[active_actuator_idx_];
            RCLCPP_DEBUG(this->get_logger(), "구동기 %d의 현재 온도: %.2f°C",
                active_actuator_idx_, current_temp_);

            // 온도 안전성 검사
            check_temperature_safety();
        } else {
            RCLCPP_WARN(this->get_logger(), "구동기 %d의 온도 데이터가 없습니다", active_actuator_idx_);
        }
    }

    // 직접 PWM 명령 수신 콜백
    void direct_pwm_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중에는 명령 무시
        if (is_emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "비상 정지 중입니다. PWM 명령이 무시됩니다.");
            return;
        }

        // 자동 모드일 때는 명령 무시
        if (!use_direct_pwm_) {
            RCLCPP_INFO(this->get_logger(), "자동 제어 모드에서 직접 PWM 명령이 수신되었습니다. 자동 모드에서는 수동 제어가 무시됩니다.");
            return;
        }

        // 해당 구동기의 PWM 값 가져오기
        if (msg->pwm.size() > static_cast<size_t>(active_actuator_idx_)) {
            direct_pwm_value_ = msg->pwm[active_actuator_idx_];
            RCLCPP_INFO(this->get_logger(),
                "직접 PWM 제어 명령 수신: 구동기 %d, PWM 값: %d",
                active_actuator_idx_, direct_pwm_value_);

            // 직접 제어 모드에서는 명령 즉시 적용
            current_pwm_values_[active_actuator_idx_] = direct_pwm_value_;
            publish_pwm_command();
        } else {
            RCLCPP_WARN(this->get_logger(),
                "유효하지 않은 PWM 명령: 구동기 %d의 데이터가 없습니다", active_actuator_idx_);
        }
    }

    // 온도 안전성 검사
    void check_temperature_safety()
    {
        double safety_threshold = this->get_parameter("safety_threshold").as_double();

        // 온도가 임계값을 초과하면 비상 정지
        if (current_temp_ >= safety_threshold && is_temperature_safe_) {
            is_temperature_safe_ = false;
            RCLCPP_ERROR(this->get_logger(),
                "온도가 안전 임계값(%.1f°C)을 초과했습니다! 현재 온도: %.1f°C",
                safety_threshold, current_temp_);

            // 비상 정지 활성화
            activate_emergency_stop();
        }
    }

    // 목표 온도 설정 콜백
    void target_temp_callback(const wearable_robot_interfaces::msg::TemperatureData::SharedPtr msg)
    {

        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중에는 목표 온도 변경 무시
        if (is_emergency_stop_) {
            RCLCPP_WARN(this->get_logger(), "비상 정지 중입니다. 목표 온도 변경이 무시됩니다.");
            return;
        }

        // 수동 제어 모드인 경우 정보 메시지만 출력하고 계속 진행
        if (use_direct_pwm_) {
            RCLCPP_INFO(this->get_logger(),
                "수동 제어 모드에서 목표 온도가 설정되었습니다. 온도 제어는 자동 모드에서만 활성화됩니다.");
        }

        // 목표 온도 업데이트 (수동 모드여도 저장은 함)
        if (msg->temperature.size() > static_cast<size_t>(active_actuator_idx_)) {
            double target_temp = msg->temperature[active_actuator_idx_];

            // 목표 온도 업데이트
            this->set_parameter(rclcpp::Parameter("target_temperature", target_temp));
            RCLCPP_INFO(this->get_logger(), "목표 온도가 %.1f°C로 업데이트되었습니다", target_temp);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "유효하지 않은 목표 온도 메시지: 구동기 %d의 데이터가 없습니다", active_actuator_idx_);
        }
    }

    // 비상 정지 활성화
    void activate_emergency_stop()
    {
        is_emergency_stop_ = true;
        RCLCPP_ERROR(this->get_logger(), "비상 정지가 활성화되었습니다! 모든 출력이 중단됩니다.");

        // PWM 출력을 0으로 설정하고 발행
        for (auto& value : current_pwm_values_) {
            value = 0;
        }
        publish_pwm_command();

        // 적분기 초기화
        integral_ = 0.0;
    }

    // 주기적 제어 콜백
    void control_callback()
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        // 비상 정지 중일 때는 제어 스킵, 0 값만 발행
        if (is_emergency_stop_) {
            // 모든 PWM 값 0으로 설정
            for (auto& value : current_pwm_values_) {
                value = 0;
            }
            publish_pwm_command();
            return;
        }

        // 직접 PWM 제어 모드일 경우 (PWM 명령은 콜백에서 즉시 처리하므로 여기서는 추가 처리 없음)
        if (use_direct_pwm_) {
            RCLCPP_DEBUG(this->get_logger(),
                "직접 PWM 제어 모드: 구동기 %d, PWM 값: %d",
                active_actuator_idx_, current_pwm_values_[active_actuator_idx_]);

            // 현재 상태 발행 (필수는 아니지만 상태 모니터링을 위해 주기적으로 발행)
            publish_pwm_command();
        }
        // 자동 온도 제어 모드일 경우
        else {
            // 파라미터 불러오기
            double target_temp = this->get_parameter("target_temperature").as_double();
            double kp = this->get_parameter("kp").as_double();
            double ki = this->get_parameter("ki").as_double();
            double max_pwm = this->get_parameter("max_pwm").as_double();
            double min_pwm = this->get_parameter("min_pwm").as_double();
            double control_frequency = this->get_parameter("control_frequency").as_double();
            double dt = 1.0 / control_frequency;

            // PI 제어 연산
            double error = target_temp - current_temp_;
            integral_ += error * dt;

            // Anti-windup
            if (integral_ > max_pwm / ki) integral_ = max_pwm / ki;
            if (integral_ < min_pwm / ki) integral_ = min_pwm / ki;

            // PI 출력 계산
            double output = kp * error + ki * integral_;

            // 출력 제한
            if (output > max_pwm) output = max_pwm;
            if (output < min_pwm) output = min_pwm;

            // 정수화
            uint8_t pwm_value = static_cast<uint8_t>(output);

            // 액티브 액추에이터 PWM 값 설정
            current_pwm_values_[active_actuator_idx_] = pwm_value;

            RCLCPP_DEBUG(this->get_logger(),
                "온도 제어: 목표=%.1f°C, 현재=%.1f°C, 오차=%.1f°C, 출력=%d",
                target_temp, current_temp_, error, pwm_value);

            // PWM 명령 발행
            publish_pwm_command();
        }
    }

    // PWM 명령 발행 함수
    void publish_pwm_command()
    {
        auto pwm_msg = wearable_robot_interfaces::msg::ActuatorCommand();
        pwm_msg.header.stamp = this->now();
        pwm_msg.pwm = current_pwm_values_;

        // 메시지 발행
        pwm_publisher_->publish(pwm_msg);
    }

    void handle_control_mode(
        const std::shared_ptr<wearable_robot_interfaces::srv::SetControlMode::Request> request,
        std::shared_ptr<wearable_robot_interfaces::srv::SetControlMode::Response> response)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        try {
            // true: 자동 모드, false: 수동 모드
            bool auto_mode = request->auto_mode;
            use_direct_pwm_ = !auto_mode;  // 자동 모드의 반대가 수동 모드

            // 모드 전환 시 적분 누적값 초기화
            if (auto_mode) {
                integral_ = 0.0;
            }

            response->success = true;
            response->message = "제어 모드가 " + std::string(auto_mode ? "자동 온도 제어" : "수동 PWM 제어") +
                               "로 변경되었습니다";

            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "제어 모드 변경 실패: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // PI 파라미터 설정 서비스 핸들러
    void handle_pi_parameters(
        const std::shared_ptr<wearable_robot_interfaces::srv::SetControlParams::Request> request,
        std::shared_ptr<wearable_robot_interfaces::srv::SetControlParams::Response> response)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        try {
            // 파라미터 유효성 검사
            if (request->kp < 0.0 || request->ki < 0.0) {
                throw std::invalid_argument("게인 값은 음수가 될 수 없습니다");
            }

            // 파라미터 업데이트
            this->set_parameter(rclcpp::Parameter("kp", request->kp));
            this->set_parameter(rclcpp::Parameter("ki", request->ki));

            response->success = true;
            response->message = "PI 파라미터가 업데이트되었습니다 (Kp=" +
                               std::to_string(request->kp) + ", Ki=" +
                               std::to_string(request->ki) + ")";

            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "PI 파라미터 업데이트 실패: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    // 비상 정지 서비스 핸들러
    void handle_emergency_stop(
        const std::shared_ptr<wearable_robot_interfaces::srv::EmergencyStop::Request> request,
        std::shared_ptr<wearable_robot_interfaces::srv::EmergencyStop::Response> response)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);

        try {
            if (request->activate) {
                // 비상 정지 활성화
                if (!is_emergency_stop_) {
                    activate_emergency_stop();
                    response->message = "비상 정지가 활성화되었습니다";
                } else {
                    response->message = "비상 정지가 이미 활성화되어 있습니다";
                }
            } else {
                // 비상 정지 해제
                if (is_emergency_stop_) {
                    is_emergency_stop_ = false;
                    response->message = "비상 정지가 해제되었습니다";

                    // 온도가 안전하면 안전 상태도 초기화
                    if (current_temp_ < this->get_parameter("safety_threshold").as_double()) {
                        is_temperature_safe_ = true;
                    } else {
                        response->message += " (주의: 온도가 여전히 안전 임계값을 초과함)";
                    }
                } else {
                    response->message = "비상 정지가 이미 해제되어 있습니다";
                }
            }

            response->success = true;
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "비상 정지 상태 변경 실패: " + std::string(e.what());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }


    // 파라미터 설정 콜백
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters) {
            if (param.get_name() == "target_temperature") {
                RCLCPP_INFO(this->get_logger(),
                    "목표 온도가 %.1f°C로 변경되었습니다", param.as_double());
            } else if (param.get_name() == "kp") {
                RCLCPP_INFO(this->get_logger(),
                    "Kp 게인이 %.2f로 변경되었습니다", param.as_double());
            } else if (param.get_name() == "ki") {
                RCLCPP_INFO(this->get_logger(),
                    "Ki 게인이 %.3f로 변경되었습니다", param.as_double());
            } else if (param.get_name() == "safety_threshold") {
                RCLCPP_INFO(this->get_logger(),
                    "안전 온도 임계값이 %.1f°C로 변경되었습니다", param.as_double());
            } else if (param.get_name() == "active_actuator") {
                active_actuator_idx_ = param.as_int();
                RCLCPP_INFO(this->get_logger(),
                    "액티브 액추에이터가 %d번으로 변경되었습니다 (actuator 0 ~ 5)",
                    active_actuator_idx_);
            }
        }

        return result;
    }

    // 구독 및 발행
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr target_temp_subscription_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr direct_pwm_subscription_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_publisher_;

    // 서비스
    rclcpp::Service<wearable_robot_interfaces::srv::SetControlMode>::SharedPtr control_mode_service_;
    rclcpp::Service<wearable_robot_interfaces::srv::SetControlParams>::SharedPtr pi_param_service_;
    rclcpp::Service<wearable_robot_interfaces::srv::EmergencyStop>::SharedPtr emergency_service_;


    // 파라미터 콜백
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 타이머
    rclcpp::TimerBase::SharedPtr control_timer_;

    // 제어 변수
    double current_temp_;       // 현재 온도
    double previous_error_;     // 이전 오차
    double integral_;           // 적분 누적값
    uint8_t pwm_output_;        // 현재 PWM 출력값
    uint8_t direct_pwm_value_;  // 직접 제어 모드 PWM 값
    bool use_direct_pwm_;       // 직접 PWM 제어 모드 사용 여부 (true: 수동 모드, false: 자동 모드)
    int active_actuator_idx_;   // 활성 액추에이터 인덱스 (0부터 시작)
    std::vector<uint8_t> current_pwm_values_;  // 현재 PWM 값 배열
    std::mutex control_mutex_;  // 제어 로직 동기화를 위한 뮤텍스

    // 안전 관련 변수
    bool is_emergency_stop_;    // 비상 정지 상태
    bool is_temperature_safe_;  // 온도 안전 상태
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActuatorControlNode>());
    rclcpp::shutdown();
    return 0;
}
