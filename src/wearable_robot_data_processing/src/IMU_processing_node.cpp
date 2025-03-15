#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/imu_data.hpp>
#include <wearable_robot_interfaces/msg/imu_type.hpp>
#include <deque>
#include <array>

class IMUProcessingNode : public rclcpp::Node
{
public:
    static const int FILTER_SIZE = 5;

    IMUProcessingNode() : Node("imu_processing_node")
    {
        subscription_ = this->create_subscription<wearable_robot_interfaces::msg::IMUData>(
            "imu_raw_data", 10,
            std::bind(&IMUProcessingNode::imu_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<wearable_robot_interfaces::msg::IMUData>(
            "imu_data", 10);

        // Initialize filter queues for each IMU's RPY values
        for (int i = 0; i < 4; ++i) {
            imu_queues_.push_back(std::array<std::deque<int32_t>, 3>());
        }

        RCLCPP_INFO(this->get_logger(), "IMU Processing Node has been started");
    }

private:
    int32_t filterIMUValue(int32_t raw_value, std::deque<int32_t>& queue) {
        queue.push_back(raw_value);
        if (queue.size() > FILTER_SIZE) {
            queue.pop_front();
        }

        int32_t sum = 0;
        for (const auto& val : queue) {
            sum += val;
        }
        return sum / queue.size();
    }

    void processIMUUnit(wearable_robot_interfaces::msg::IMUType& output_imu,
                       const wearable_robot_interfaces::msg::IMUType& input_imu,
                       std::array<std::deque<int32_t>, 3>& queues) {
        output_imu.roll = filterIMUValue(input_imu.roll, queues[0]);
        output_imu.pitch = filterIMUValue(input_imu.pitch, queues[1]);
        output_imu.yaw = filterIMUValue(input_imu.yaw, queues[2]);
    }

    void imu_callback(const wearable_robot_interfaces::msg::IMUData::SharedPtr msg)
    {
        auto processed_msg = wearable_robot_interfaces::msg::IMUData();
        processed_msg.header = msg->header;


        // Process each IMU
        processIMUUnit(processed_msg.imu1, msg->imu1, imu_queues_[0]);
        processIMUUnit(processed_msg.imu2, msg->imu2, imu_queues_[1]);
        processIMUUnit(processed_msg.imu3, msg->imu3, imu_queues_[2]);
        processIMUUnit(processed_msg.imu4, msg->imu4, imu_queues_[3]);

        publisher_->publish(processed_msg);

        RCLCPP_DEBUG(this->get_logger(),
            "Processed IMU1 data - Roll: %d, Pitch: %d, Yaw: %d",
            processed_msg.imu1.roll,
            processed_msg.imu1.pitch,
            processed_msg.imu1.yaw);
    }

    rclcpp::Subscription<wearable_robot_interfaces::msg::IMUData>::SharedPtr subscription_;
    rclcpp::Publisher<wearable_robot_interfaces::msg::IMUData>::SharedPtr publisher_;
    std::vector<std::array<std::deque<int32_t>, 3>> imu_queues_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
