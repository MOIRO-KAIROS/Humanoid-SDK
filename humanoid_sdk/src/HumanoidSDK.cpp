#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "HumanoidSDK.hpp"

HumanoidSDK::HumanoidSDK(const std::string& device_name, float protocol_version, int baudrate) {
    portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(protocol_version);

    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to open the port!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to open the port.");
    }

    dxl_comm_result = portHandler->setBaudRate(baudrate);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to set the baudrate!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to set the baudrate.");
    }
}

HumanoidSDK::~HumanoidSDK() {
    portHandler->closePort();
}

void HumanoidSDK::DynamixelSetup(uint8_t dxl_id) {
    setupDynamixel(dxl_id);
}

void HumanoidSDK::DynamixelWrite(uint8_t dxl_id, uint32_t goal_position) {
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

uint16_t HumanoidSDK::DynamixelRead(uint8_t dxl_id) {
    uint16_t present_position = 0;

    dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_PRESENT_POSITION,
        &present_position,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "%s", packetHandler->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Get [ID: %d] [Present Position: %d]", dxl_id, present_position);
    }

    return present_position;
}

void HumanoidSDK::DynamixelClose(uint8_t dxl_id) {
    closeDynamixel(dxl_id);
}

void HumanoidSDK::setupDynamixel(uint8_t dxl_id) {
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

void HumanoidSDK::closeDynamixel(uint8_t dxl_id) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        dxl_id,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Failed to disable torque for ID: %d", dxl_id);
    } else {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidSDK"), "Succeeded to disable torque for ID: %d", dxl_id);
    }
}

void HumanoidSDK::MultiDynamixelSetup(const std::vector<uint8_t>& ids) {
    for (size_t i = 0; i < ids.size(); ++i) {
        DynamixelSetup(ids[i]);
    }
}

void HumanoidSDK::MultiDynamixelWrite(const std::vector<uint8_t>& ids, const std::vector<uint32_t>& goal_positions) {
    if (ids.size() != goal_positions.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidSDK"), "Mismatched sizes: ids and goal_positions must have the same length.");
        return;
    }

    for (size_t i = 0; i < ids.size(); ++i) {
        DynamixelWrite(ids[i], goal_positions[i]);
    }
}

std::vector<uint16_t> HumanoidSDK::MultiDynamixelRead(const std::vector<uint8_t>& ids) {
    std::vector<uint16_t> present_positions;

    for (size_t i = 0; i < ids.size(); ++i) {
        uint16_t position = DynamixelRead(ids[i]);
        present_positions.push_back(position);
    }

    return present_positions;
}

void HumanoidSDK::MultiDynamixelClose(const std::vector<uint8_t>& ids) {
    for (size_t i = 0; i < ids.size(); ++i) {
        DynamixelClose(ids[i]);
    }
}