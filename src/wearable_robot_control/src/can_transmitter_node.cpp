#include <rclcpp/rclcpp.hpp>
#include <wearable_robot_interfaces/msg/actuator_command.hpp>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>

// SocketCAN 관련 헤더 추가
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>  // close()
#include <cstring>   // strcpy, memset 함수 사용을 위해 추가

// 스레드 안전 큐 구현
template<typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() {}
    ~ThreadSafeQueue() {}

    // 큐에 항목 추가
    void push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(item);
        lock.unlock();
        cv_.notify_one();  // 대기 중인 스레드에 알림
    }

    // 큐에서 항목 가져오기 (블로킹)
    T pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || shutdown_; });

        if (shutdown_ && queue_.empty()) {
            return T();  // 종료 시 빈 객체 반환
        }

        T item = queue_.front();
        queue_.pop();
        return item;
    }

    // 큐 종료 신호
    void shutdown() {
        std::unique_lock<std::mutex> lock(mutex_);
        shutdown_ = true;
        lock.unlock();
        cv_.notify_all();  // 모든 대기 스레드에 알림
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_ = false;
};

// CANFD 프레임 구조체
struct CANFDFrame {
    canid_t id;
    std::vector<uint8_t> data;
};

class CANTransmitterNode : public rclcpp::Node
{
public:
    CANTransmitterNode() : Node("can_transmitter_node"), shutdown_(false)
    {
        // PWM 명령 구독
        pwm_subscription_ = this->create_subscription<wearable_robot_interfaces::msg::ActuatorCommand>(
            "actuator_command", 10,
            std::bind(&CANTransmitterNode::pwm_command_callback, this, std::placeholders::_1));

        // CAN 인터페이스 파라미터
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("can_id", "400");  // 기본 ID 0x400 (1024)

        can_interface_ = this->get_parameter("can_interface").as_string();
        can_id_ = std::stoi(this->get_parameter("can_id").as_string(), nullptr, 16);

        RCLCPP_INFO(this->get_logger(), "CAN 송신 노드가 시작되었습니다");
        RCLCPP_INFO(this->get_logger(), "CAN 인터페이스: %s, CAN ID: 0x%x",
            can_interface_.c_str(), can_id_);

        // CAN 통신용 스레드 시작
        can_thread_ = std::thread(&CANTransmitterNode::can_thread_func, this);
    }

    ~CANTransmitterNode() {
        // 종료 시 정리 작업
        shutdown_ = true;
        can_queue_.shutdown();
        if (can_thread_.joinable()) {
            can_thread_.join();  // 스레드 종료 대기
        }
        if (can_socket_ >= 0) {
            close(can_socket_);  // 소켓 닫기
        }
    }

private:
    // PWM 명령 수신 및 CAN 전송 콜백
    void pwm_command_callback(const wearable_robot_interfaces::msg::ActuatorCommand::SharedPtr msg)
    {
        // CAN 데이터 생성
        std::vector<uint8_t> data(64, 0);
        // PWM 값을 CAN 데이터에 복사
        for (size_t i = 0; i < msg->pwm.size() && i < 6; ++i) {
            data[i] = msg->pwm[i];
        }

        // CAN 메시지 큐에 전송 (비동기 처리)
        CANFDFrame frame;
        frame.id = can_id_;
        frame.data = data;
        can_queue_.push(frame);

        RCLCPP_DEBUG(this->get_logger(), "CAN 메시지가 큐에 추가됨: ID 0x%x, 데이터 크기: %zu",
            frame.id, frame.data.size());

        // 여기서 함수가 바로 반환되므로, 메인 스레드는 CAN 전송을 기다리지 않고 계속 실행됨
    }

    // CAN 통신 스레드 함수
    void can_thread_func() {
        // SocketCAN 초기화
        if (!init_can_socket()) {
            RCLCPP_ERROR(this->get_logger(), "CAN 소켓 초기화 실패, 스레드 종료");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "CAN 통신 스레드가 시작되었습니다");

        while (!shutdown_) {
            // 큐에서 메시지 가져오기 (블로킹 호출)
            CANFDFrame frame = can_queue_.pop();

            // 종료 신호 확인
            if (shutdown_) {
                break;
            }

            // CANFD 메시지 전송
            send_can_message(frame);
        }

        RCLCPP_INFO(this->get_logger(), "CAN 통신 스레드가 종료되었습니다");
    }

    // SocketCAN 초기화
    bool init_can_socket() {
        // 소켓 생성
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 소켓 생성 실패: %s", strerror(errno));
            return false;
        }

        // CANFD 모드 활성화
        int enable_canfd = 1;
        if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CANFD 모드 활성화 실패: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        // 인터페이스 인덱스 가져오기
        struct ifreq ifr;
        strcpy(ifr.ifr_name, can_interface_.c_str());
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 인터페이스 인덱스 가져오기 실패: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        // 소켓 바인딩
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 소켓 바인딩 실패: %s", strerror(errno));
            close(can_socket_);
            can_socket_ = -1;
            return false;
        }

        return true;
    }

    // CAN 메시지 전송 함수
    void send_can_message(const CANFDFrame& frame) {
        if (can_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN 소켓이 초기화되지 않았습니다");
            return;
        }

        // CANFD 프레임 구성
        struct canfd_frame cf;
        memset(&cf, 0, sizeof(cf));
        cf.can_id = frame.id;
        cf.len = frame.data.size();

        // 데이터 복사
        for (size_t i = 0; i < frame.data.size() && i < 64; ++i) {
            cf.data[i] = frame.data[i];
        }

        // 프레임 전송
        int nbytes = write(can_socket_, &cf, sizeof(struct canfd_frame));
        if (nbytes != sizeof(struct canfd_frame)) {
            RCLCPP_ERROR(this->get_logger(),
                "CAN 메시지 전송 실패: 오류 코드: %s", strerror(errno));
        } else {
            RCLCPP_DEBUG(this->get_logger(),
                "CAN 메시지 전송 성공: ID 0x%x, 데이터 크기: %zu", frame.id, frame.data.size());
        }
    }

    // 구독
    rclcpp::Subscription<wearable_robot_interfaces::msg::ActuatorCommand>::SharedPtr pwm_subscription_;

    // CAN 통신 관련 변수
    std::string can_interface_;
    canid_t can_id_;
    int can_socket_ = -1;

    // 비동기 처리를 위한 변수
    ThreadSafeQueue<CANFDFrame> can_queue_;
    std::thread can_thread_;
    std::atomic<bool> shutdown_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANTransmitterNode>());
    rclcpp::shutdown();
    return 0;
}
