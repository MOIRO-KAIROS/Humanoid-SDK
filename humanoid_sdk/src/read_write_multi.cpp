#include "HumanoidSDK.cpp"
#include <iostream>
#include <chrono>
#include <thread>

// Default setting
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyAMA0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<HumanoidSDK>(DEVICE_NAME, PROTOCOL_VERSION, BAUDRATE);

    // Setup Multi Dynamixel motors
    std::vector<uint8_t> ids = {11, 17};
    controller->MultiDynamixelSetup(ids);

    // Write to Dynamixel
    controller->MultiDynamixelWrite(ids, {1000, 2000});

    // Read from Dynamixel
    std::vector<uint16_t> positions = controller->MultiDynamixelRead(ids);

    // Close specific Dynamixel motors
    controller->MultiDynamixelClose(ids);

    rclcpp::shutdown();
    return 0;
}