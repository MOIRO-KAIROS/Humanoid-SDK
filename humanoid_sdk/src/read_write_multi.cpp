// ros2 run humanoid_sdk read_write_multi --ros-args -p id:=[11] -p goal_position:=[1500]

#include "rclcpp/rclcpp.hpp"
#include "HumanoidSDK.cpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include "util.cpp"

#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.
#define BAUDRATE 1000000      // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyAMA0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

class MultiReadAndWriteNode : public rclcpp::Node {
public:
    MultiReadAndWriteNode()
    : Node("read_and_write_multi")
    {
        // ROS2 param
        this->declare_parameter<std::vector<int64_t>>("id", {11, 17});
        this->declare_parameter<std::vector<int64_t>>("goal_position", {1000, 1000});
        
        auto motor_ids_temp = this->get_parameter("id").as_integer_array();
        auto goal_positions_temp = this->get_parameter("goal_position").as_integer_array();
        
        // 타입 변환
        motor_ids_ = convert_to_uint8(motor_ids_temp);
        goal_positions_ = convert_to_uint32(goal_positions_temp);

        // Dynamixel 제어를 위한 컨트롤러 객체 생성
        controller_ = std::make_shared<HumanoidSDK>(DEVICE_NAME, PROTOCOL_VERSION, BAUDRATE);
        if (!controller_->is_initialized) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller. Shutting down node.");
            rclcpp::shutdown();
            return;
        }
        // Dynamixel 모터 설정
        controller_->MultiDynamixelSetup(motor_ids_);

        // 주기적으로 Dynamixel 모터의 위치를 읽고 쓰는 타이머 생성 (1초 주기)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&MultiReadAndWriteNode::control_loop, this));
    }

    ~MultiReadAndWriteNode()
    {
        // 모터 자원 해제
        controller_->MultiDynamixelClose(motor_ids_);
    }

private:
    void control_loop()
    {
        if (!controller_) {
            RCLCPP_WARN(this->get_logger(), "Controller not initialized. Skipping control loop.");
            return;
        }
        // 각 모터 ID에 해당하는 목표 위치 쓰기
        controller_->MultiDynamixelWrite(motor_ids_, goal_positions_);

        // 각 모터의 현재 위치 읽기
        std::vector<uint16_t> positions = controller_->MultiDynamixelRead(motor_ids_);

        // 각 모터의 현재 위치를 로깅
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Motor ID %d position: %d", motor_ids_[i], positions[i]);
        }
    }

    std::shared_ptr<HumanoidSDK> controller_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<uint8_t> motor_ids_;
    std::vector<uint32_t> goal_positions_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiReadAndWriteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}