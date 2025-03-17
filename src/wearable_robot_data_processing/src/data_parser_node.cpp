#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/can_data_frame.hpp>
#include <wearable_robot_interfaces/msg/displacement_raw_data.hpp>
#include <wearable_robot_interfaces/msg/imu_data.hpp>
#include <wearable_robot_interfaces/msg/imu_type.hpp>
#include <wearable_robot_interfaces/msg/temperature_data.hpp>
#include <wearable_robot_interfaces/msg/fan_command.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <array>
#include <vector>

class DataParserNode : public rclcpp::Node
{
public:
    DataParserNode() : Node("data_parser_node")
    {
        // CAN 데이터 구독
        subscription_100_ = this->create_subscription<wearable_robot_interfaces::msg::CANDataFrame>(
            "can_data_100", 100,
            std::bind(&DataParserNode::CAN_data_100_callback, this, std::placeholders::_1));

        subscription_401_ = this->create_subscription<wearable_robot_interfaces::msg::CANDataFrame>(
            "can_data_401", 100,
            std::bind(&DataParserNode::CAN_data_401_callback, this, std::placeholders::_1));

        // 발행자 설정
        displacement_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::DisplacementRawData>(
            "displacement_raw_data", 10);
        imu_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::IMUData>(
            "imu_raw_data", 10);
        temp_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::TemperatureData>(
            "temperature_data", 10);
        fan_state_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::FanCommand>(
            "fan_state", 10);
        pwm_state_publisher_ = this->create_publisher<wearable_robot_interfaces::msg::ActuatorCommand>(
            "pwm_state", 10);
    }

private:
    static constexpr size_t NUM_DISPLACEMENT_SENSORS = 10;
    static constexpr size_t NUM_IMU_SENSORS = 4;
    static constexpr size_t NUM_TEMPERATURE_SENSORS = 6;
    static constexpr size_t NUM_PWM_CHANNELS = 6;
    static constexpr size_t NUM_FAN_CHANNELS = 6;

    void CAN_data_100_callback(const wearable_robot_interfaces::msg::CANDataFrame::SharedPtr msg)
    {
        // 변위 센서 데이터 처리
        auto displacement_msg = wearable_robot_interfaces::msg::DisplacementRawData();
        displacement_msg.header = msg->header;

        // 변위 센서 데이터 배열로 할당
        displacement_msg.displacement_raw.resize(NUM_DISPLACEMENT_SENSORS);
        for (size_t i = 0; i < NUM_DISPLACEMENT_SENSORS; ++i) {
            size_t data_index = i * 2;
            uint16_t raw_displacement = (static_cast<uint16_t>(msg->data[data_index + 1]) << 8) |
            static_cast<uint16_t>(msg->data[data_index]);
            displacement_msg.displacement_raw[i] = static_cast<int>(raw_displacement);
        }

        // IMU 데이터 처리
        auto imu_msg = wearable_robot_interfaces::msg::IMUData();
        imu_msg.header = msg->header;

        // IMU 데이터 배열로 처리
        std::array<wearable_robot_interfaces::msg::IMUType*, NUM_IMU_SENSORS> imus = {
            &imu_msg.imu1, &imu_msg.imu2, &imu_msg.imu3, &imu_msg.imu4
        };

        for (size_t i = 0; i < NUM_IMU_SENSORS; ++i) {
            size_t base_idx = (NUM_DISPLACEMENT_SENSORS*2) + (i * 3);
            imus[i]->roll = msg->data[base_idx];
            imus[i]->pitch = msg->data[base_idx + 1];
            imus[i]->yaw = msg->data[base_idx + 2];
        }

        // 메시지 발행
        displacement_publisher_->publish(displacement_msg);
        imu_publisher_->publish(imu_msg);
    }

    void CAN_data_401_callback(const wearable_robot_interfaces::msg::CANDataFrame::SharedPtr msg)
    {
        auto time_stamp = this->now();

        // PWM 상태 처리
        auto pwm_msg = wearable_robot_interfaces::msg::ActuatorCommand();
        pwm_msg.header.stamp = time_stamp;
        pwm_msg.pwm.resize(NUM_PWM_CHANNELS);
        for (size_t i = 0; i < NUM_PWM_CHANNELS; ++i) {
            pwm_msg.pwm[i] = msg->data[i];
        }

        // FAN 상태 처리
        auto fan_msg = wearable_robot_interfaces::msg::FanCommand();
        fan_msg.header.stamp = time_stamp;
        fan_msg.fan.resize(NUM_FAN_CHANNELS);
        for (size_t i = 0; i < NUM_FAN_CHANNELS; ++i) {
            fan_msg.fan[i] = msg->data[i + NUM_PWM_CHANNELS] > 0;
        }

        // 온도 데이터 처리
        auto temp_msg = wearable_robot_interfaces::msg::TemperatureData();
        temp_msg.header.stamp = time_stamp;
        temp_msg.temperature.resize(NUM_TEMPERATURE_SENSORS);

        for (size_t i = 0; i < NUM_TEMPERATURE_SENSORS; ++i) {
            size_t data_index = NUM_PWM_CHANNELS + NUM_FAN_CHANNELS + (i * 2);
            uint16_t raw_temp = (static_cast<uint16_t>(msg->data[data_index + 1]) << 8) |
                                static_cast<uint16_t>(msg->data[data_index]);
            temp_msg.temperature[i] = static_cast<float>(raw_temp) * 0.25f;
            if (temp_msg.temperature[i] > 500.0f) {
                temp_msg.temperature[i] = 0.0f;
            }
        }

        // 메시지 발행
        pwm_state_publisher_->publish(pwm_msg);
        fan_state_publisher_->publish(fan_msg);
        temp_publisher_->publish(temp_msg);

        RCLCPP_DEBUG(this->get_logger(),
            "Parsed CAN 0x401 data - PWM[0]: %d, FAN[0]: %d, TEMP[0]: %.2f",
            pwm_msg.pwm[0], fan_msg.fan[0], temp_msg.temperature[0]);
    }

    rclcpp::Subscription<wearable_robot_interfaces::msg::CANDataFrame>::SharedPtr subscription_100_;
    rclcpp::Subscription<wearable_robot_interfaces::msg::CANDataFrame>::SharedPtr subscription_401_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::DisplacementRawData>::SharedPtr displacement_publisher_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::IMUData>::SharedPtr imu_publisher_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::TemperatureData>::SharedPtr temp_publisher_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::FanCommand>::SharedPtr fan_state_publisher_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_state_publisher_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataParserNode>());
    rclcpp::shutdown();
    return 0;
}
