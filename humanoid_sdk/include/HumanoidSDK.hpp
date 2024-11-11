#ifndef HUMANOID_SDK_HPP
#define HUMANOID_SDK_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

class HumanoidSDK {
public:
    HumanoidSDK(const std::string& device_name, float protocol_version, int baudrate);
    ~HumanoidSDK();
    
    void DynamixelSetup(uint8_t dxl_id);
    void DynamixelWrite(uint8_t dxl_id, uint32_t goal_position);
    uint16_t DynamixelRead(uint8_t dxl_id);
    void DynamixelClose(uint8_t dxl_id);

    // Multi-motor operations
    void MultiDynamixelSetup(const std::vector<uint8_t>& ids);
    void MultiDynamixelWrite(const std::vector<uint8_t>& ids, const std::vector<uint32_t>& goal_positions);
    std::vector<uint16_t> MultiDynamixelRead(const std::vector<uint8_t>& ids);
    void MultiDynamixelClose(const std::vector<uint8_t>& ids);


private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    uint8_t dxl_error;
    int dxl_comm_result;
    
    void setupDynamixel(uint8_t dxl_id);
    void closeDynamixel(uint8_t dxl_id);
};

#endif  // HUMANOID_SDK_HPP