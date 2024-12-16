// ros2 run humanoid_sdk read_write_node --ros-args -p id:=10
// ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 10, position: 503}"
// ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 10"

#include "rclcpp/rclcpp.hpp"
#include "HumanoidSDK.cpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rcutils/cmdline_parser.h"

#include <iostream>
#include <chrono>
#include <thread>

// Default setting
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.
#define BAUDRATE 1000000      // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

using SetPosition = dynamixel_sdk_custom_interfaces::msg::SetPosition;
using GetPosition = dynamixel_sdk_custom_interfaces::srv::GetPosition;

class ReadAndWriteNode : public rclcpp::Node {
public:
    ReadAndWriteNode()
    : Node("read_and_write_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing read_and_write_node...");

        // ROS2 param
		this->declare_parameter("qos_depth", 10);
        int8_t qos_depth = 0;
        this->get_parameter("qos_depth", qos_depth);
		
        this->declare_parameter<int>("id", 10);
        // this->declare_parameter<int>("goal_position", 1000);

        id = this->get_parameter("id").as_int();
        // goal_position_ = this->get_parameter("goal_position").as_int();

		const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

		// Dynamixel 제어를 위한 컨트롤러 객체 생성
        controller = std::make_shared<HumanoidSDK>(DEVICE_NAME, PROTOCOL_VERSION, BAUDRATE);
        if (controller->getDxlStatus()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize controller. Shutting down node.");
            rclcpp::shutdown();
            return ;
        }

        // Dynamixel 모터 설정
        controller->SetupDynamixel(id);

		set_position_subscriber = 
			this->create_subscription<SetPosition>(
			"set_position",
			QOS_RKL10V,
			[this](const SetPosition::SharedPtr msg) -> void {
				if (!controller->getDxlStatus()) {
					RCLCPP_WARN(this->get_logger(), "Controller not initialized. Skipping set_goal_position.");
					return ;
				}

				// For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
				controller->WriteGoalPosition((uint8_t)msg->id, (uint16_t) msg->position);
			}
		);
        
		auto get_present_position =
			[this](
			const std::shared_ptr<GetPosition::Request> request,
			std::shared_ptr<GetPosition::Response> response) -> void {
				// When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
				uint16_t present_position = controller->ReadGoalPosition((uint8_t) request->id);
				response->position = present_position;
		};

		get_position_server = create_service<GetPosition>("get_position", get_present_position);
    }

    ~ReadAndWriteNode()
    {
        // 모터 자원 해제
        controller->CloseDynamixel(10);
    }

private:
	std::shared_ptr<HumanoidSDK> controller;
    rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber;
  	rclcpp::Service<GetPosition>::SharedPtr get_position_server;
    int id;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReadAndWriteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
