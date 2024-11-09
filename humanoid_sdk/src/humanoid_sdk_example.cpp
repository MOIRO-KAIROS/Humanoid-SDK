#include <cstdio>
#include <memory>
#include <string>

#include "../include/humanoid_sdk/HumanoidSDK.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

class HumanoidSDKExample : public rclcpp::Node
{
public:
    HumanoidSDKExample()
    : Node("humanoid_sdk_example")
    {
        RCLCPP_INFO(this->get_logger(), "HumanoidSDKExample Node has been started.");
    }
private:
    HumanoidSDK * humanoidSDK;
};

int main(){
    rclcpp::init(0, nullptr);
    humanoidSDK = new HumanoidSDK();
    rclcpp::spin(std::make_shared<HumanoidSDK>());
    rclcpp::shutdown();
    return 0;
}