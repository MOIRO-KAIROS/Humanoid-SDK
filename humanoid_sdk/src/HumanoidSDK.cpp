#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36
// #define BROADCAST_ID 254

#include <cstdio>
#include <memory>
#include <string>
#include <map>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "HumanoidSDK.hpp"

// motor_tables = {
//     {"AX-12A", {24, 30, 36, 0}},
//     {"MX-28T", {64, 116, 132, 11}}, // 다른 모델 추가 가능
// };


HumanoidSDK::HumanoidSDK(const std::string& device_name, float protocol_version, int baudrate, const std::string& dxl_series)
    : dxl_series(dxl_series) {  // 초기화 목록에서 값 복사
    RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "SDK initialized with series: %s", this->dxl_series.c_str());

    portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);
    
    // Open Serial Port
    int is_port_opened = portHandler->openPort();
    if (dxl_comm_result == COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to open the port!");
        return ;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    int is_baudrate_set = portHandler->setBaudRate(baudrate);
    if (dxl_comm_result == COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to set the baudrate!");
        return ;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to set the baudrate.");
    }
    
    dxl_comm_result = (is_port_opened || is_baudrate_set);
}

HumanoidSDK::~HumanoidSDK() {
    portHandler->closePort();
}

void HumanoidSDK::SetupDynamixel(uint8_t dxl_id) {
    if (dxl_series == "MX-28T") {
        TurnOffTorque(dxl_id);
    }
    EnableTorque(dxl_id);
}

void HumanoidSDK::CloseDynamixel(uint8_t dxl_id) {
    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );
}

bool HumanoidSDK::getDxlStatus(){
    return !(dxl_comm_result != COMM_SUCCESS); // COMM_SUCCESS = 0
}

void HumanoidSDK::TurnOffTorque(uint8_t dxl_id) {
    // Use Position Control Mode (MX ver)
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_OPERATING_MODE,
        3,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to set Position Control Mode.");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to set Position Control Mode.");
    }
}

void HumanoidSDK::EnableTorque(uint8_t dxl_id) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        1,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to enable torque for ID: %d", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to enable torque for ID: %d", dxl_id);
    }
}

// When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
void HumanoidSDK::WriteGoalPosition(uint8_t dxl_id, uint16_t goal_position)
{
    // Write Goal Position (length : 4 bytes)
    // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
    dxl_comm_result = packetHandler->write2ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_GOAL_POSITION,
        goal_position,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "%s", packetHandler->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Set [ID: %d] [Goal Position: %d]", dxl_id, goal_position);
    }
}

// When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
uint16_t HumanoidSDK::ReadGoalPosition(uint8_t dxl_id) {
    uint16_t present_position;

    // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.  
    dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint16_t *>(&present_position),
        &dxl_error
    );
    
    RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Get [ID: %d] [Present Position: %d]", dxl_id, present_position);

    return present_position;
}

void HumanoidSDK::SetupMultiDynamixel(const std::vector<uint8_t>& ids) {
    for (size_t i = 0; i < ids.size(); ++i) {
        SetupDynamixel(ids[i]);
    }
}

void HumanoidSDK::WriteMultiGoalPosition(const std::vector<uint8_t>& ids, const std::vector<uint16_t>& goal_positions) {
    if (ids.size() != goal_positions.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Mismatched sizes: ids and goal_positions must have the same length.");
        return;
    }

    for (size_t i = 0; i < ids.size(); ++i) {
        WriteGoalPosition(ids[i], goal_positions[i]);
    }
}

std::vector<uint16_t> HumanoidSDK::ReadMultiGoalPosition(const std::vector<uint8_t>& ids) {
    std::vector<uint16_t> present_positions;

    for (size_t i = 0; i < ids.size(); ++i) {
        uint16_t position = ReadGoalPosition(ids[i]);
        present_positions.push_back(position);
    }

    return present_positions;
}

void HumanoidSDK::CloseMultiDynamixel(const std::vector<uint8_t>& ids) {
    for (size_t i = 0; i < ids.size(); ++i) {
        CloseDynamixel(ids[i]);
    }
}