#ifndef HUMANOID_SDK_HPP
#define HUMANOID_SDK_HPP

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"

// struct ControlTable {
//     uint8_t torque_enable;
//     uint8_t goal_position;
//     uint8_t present_position;
// };

class HumanoidSDK {
public:
    HumanoidSDK(const std::string& device_name, float protocol_version, int baudrate, const std::string& dxl_series = "AX-12A");
    ~HumanoidSDK();

    void SetupDynamixel(uint8_t dxl_id);
    void CloseDynamixel(uint8_t dxl_id);
    void WriteGoalPosition(uint8_t dxl_id, uint16_t goal_position);
    uint16_t ReadGoalPosition(uint8_t dxl_id);
    bool getDxlStatus();
    
    // // Multi-motor operations
    void SetupMultiDynamixel(const std::vector<uint8_t>& ids);
    void WriteMultiGoalPosition(const std::vector<uint8_t>& ids, const std::vector<uint16_t>& goal_positions);
    std::vector<uint16_t> ReadMultiGoalPosition(const std::vector<uint8_t>& ids);
    void CloseMultiDynamixel(const std::vector<uint8_t>& ids);

private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    // std::map<std::string, ControlTable> motor_tables;

    uint8_t dxl_error;
    int dxl_comm_result = COMM_TX_FAIL;
    const std::string& dxl_series;

    void TurnOffTorque(uint8_t dxl_id);
    void EnableTorque(uint8_t dxl_id);
};

#endif  // HUMANOID_SDK_HPP