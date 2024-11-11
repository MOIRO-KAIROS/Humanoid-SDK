#include "HumanoidSDK.cpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

// Default setting
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL X series.
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyAMA0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    // Create a controller object with the desired device, protocol version, and baud rate
    auto controller = std::make_shared<HumanoidSDK>(DEVICE_NAME, PROTOCOL_VERSION, BAUDRATE);

    // Setup Dynamixel motors with IDs 11 and 17
    controller->DynamixelSetup(11);
    controller->DynamixelSetup(17);

    // Write goal position to motor with ID 11
    controller->DynamixelWrite(11, 1000);

    // Read the present position from motor with ID 11
    uint16_t position = controller->DynamixelRead(11);

    // Close motor with ID 11
    controller->DynamixelClose(11);

    rclcpp::shutdown();
    return 0;
}