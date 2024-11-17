// ros2 run humanoid_sdk read_write_node --ros-args -p id:=11 -p goal_position:=1500

#include "rclcpp/rclcpp.hpp"
#include "HumanoidSDK.cpp"
#include <iostream>
#include <chrono>
#include <thread>

// Default setting
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.
#define BAUDRATE 1000000      // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

class ReadAndWriteNode : public rclcpp::Node {
public:
    ReadAndWriteNode()
    : Node("read_and_write_node")
    {
        // ROS2 param
        this->declare_parameter<int>("id", 11);
        this->declare_parameter<int>("goal_position", 1000);

        motor_id_ = this->get_parameter("id").as_int();
        goal_position_ = this->get_parameter("goal_position").as_int();

        // Dynamixel 제어를 위한 컨트롤러 객체 생성
        controller_ = std::make_shared<HumanoidSDK>(DEVICE_NAME, PROTOCOL_VERSION, BAUDRATE);
        if (!controller_->is_initialized) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller. Shutting down node.");
            rclcpp::shutdown();
            return;
        }

        // Dynamixel 모터 설정
        controller_->DynamixelSetup(motor_id_);

        // 주기적으로 Dynamixel 모터의 위치를 읽고 쓰는 타이머 생성 (1초 주기)
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&ReadAndWriteNode::control_loop, this));
    }

    ~ReadAndWriteNode()
    {
        // 모터 자원 해제
        controller_->DynamixelClose(motor_id_);
    }

private:
    void control_loop()
    {
        // // 파라미터 업데이트 확인 (구현x)
        // this->get_parameter("goal_position", goal_position_);
        if (!controller_) {
            RCLCPP_WARN(this->get_logger(), "Controller not initialized. Skipping control loop.");
            return;
        }

        // 목표 위치 쓰기
        controller_->DynamixelWrite(motor_id_, goal_position_);

        // 현재 위치 읽기
        uint16_t position = controller_->DynamixelRead(motor_id_);

        // 현재 위치를 로깅
        RCLCPP_INFO(this->get_logger(), "Motor ID %d goal position: %d, current position: %d", motor_id_, goal_position_, position);
    }

    std::shared_ptr<HumanoidSDK> controller_;
    rclcpp::TimerBase::SharedPtr timer_;
    int motor_id_;
    int goal_position_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReadAndWriteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}