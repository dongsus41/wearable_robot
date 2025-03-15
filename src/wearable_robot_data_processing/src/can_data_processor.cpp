#include <rclcpp/rclcpp.hpp>
#include <ros2_socketcan_msgs/msg/fd_frame.hpp>
#include <wearable_robot_interfaces/msg/can_data_frame.hpp>

class CanDataProcessor : public rclcpp::Node
{
public:
  CanDataProcessor()
  : Node("can_data_processor")
  {
    // CAN FD 프레임을 구독. ros2_socketcan 패키지로부터 발햄됨.
    can_sub_ = this->create_subscription<ros2_socketcan_msgs::msg::FdFrame>(
      "from_can_bus_fd", 500,
      std::bind(&CanDataProcessor::canFdCallback, this, std::placeholders::_1));

    // CAN ID 별로 topic 발행
    pub_id_100_ = this->create_publisher<wearable_robot_interfaces::msg::CANDataFrame>(
      "can_data_100", 500);
    pub_id_401_ = this->create_publisher<wearable_robot_interfaces::msg::CANDataFrame>(
      "can_data_401", 500);

    RCLCPP_INFO(this->get_logger(), "CAN FD Data Processor initialized");
  }

private:
  void canFdCallback(const ros2_socketcan_msgs::msg::FdFrame::SharedPtr msg)
  {
    wearable_robot_interfaces::msg::CANDataFrame processed_data;

    processed_data.header.stamp = msg->header.stamp;
    processed_data.header.frame_id = msg->header.frame_id;

    switch (msg->id) {
      case 0x100: {  // 500Hz, 48 bytes
        if (msg->len != 48) {
          RCLCPP_WARN(this->get_logger(),
            "Unexpected data length for ID 0x100: got %d, expected 48", msg->len);
          return;
        }

        // 48바이트 데이터를 처리
        processed_data.data.resize(48);
        for (size_t i = 0; i < 48 && i < msg->data.size(); ++i) {
          processed_data.data[i] = msg->data[i];
        }
        pub_id_100_->publish(processed_data);
        break;
      }

      case 0x401: {  // 250Hz, 24 bytes
        if (msg->len != 24) {
          RCLCPP_WARN(this->get_logger(),
            "Unexpected data length for ID 0x401: got %d, expected 24", msg->len);
          return;
        }

        // 24바이트 데이터를 처리
        processed_data.data.resize(24);
        for (size_t i = 0; i < 24 && i < msg->data.size(); ++i) {
          processed_data.data[i] = msg->data[i];
        }
        pub_id_401_->publish(processed_data);
        break;
      }

      default:
        // 다른 CAN ID는 무시. 추가 장치 발생시 코드 추가
        break;
    }
  }

  // ROS2 구독 발행
  rclcpp::Subscription<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr can_sub_;
  rclcpp::Publisher<wearable_robot_interfaces::msg::CANDataFrame>::SharedPtr pub_id_100_;
  rclcpp::Publisher<wearable_robot_interfaces::msg::CANDataFrame>::SharedPtr pub_id_401_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CanDataProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
