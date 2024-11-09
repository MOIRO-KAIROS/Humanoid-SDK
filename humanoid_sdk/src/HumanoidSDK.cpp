#include "../include/humanoid_sdk/HumanoidSDK.hpp"

HumanoidSDK::HumanoidSDK()
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open Serial Port
    int dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false)
    {
        printf("Failed to open the port!\n");
    }
    else
    {
        printf("Succeeded to open the port\n");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false)
    {
        printf("Failed to set the baudrate!\n");
    }
    else
    {
        printf("Succeeded to set the baudrate\n");
    }
}

HumanoidSDK::~HumanoidSDK()
{
    delete portHandler;
    delete packetHandler;
}

void HumanoidSDK::setupDynamixel(uint8_t dxl_id)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Enable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
        packetHandler->printRxPacketError(dxl_error);
    }
    else
    {
        printf("Dynamixel has been successfully connected\n");
    }
}

void HumanoidSDK::MultiDynamixelSetup(uint8_t * dxl_id, uint8_t dxl_cnt)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Enable Dynamixel Torque
    for (int i = 0; i < dxl_cnt; i++)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS)
        {
            packetHandler->printTxRxResult(dxl_comm_result);
        }
        else if (dxl_error != 0)
        {
            packetHandler->printRxPacketError(dxl_error);
        }
        else
        {
            printf("Dynamixel ID: %d has been successfully connected\n", dxl_id[i]);
        }
    }
}

void HumanoidSDK::writeDynamixel(uint8_t dxl_id, uint16_t addr, uint16_t goal_position)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // Write data to Dynamixel
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id, addr, goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
    } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", dxl_id, goal_position);
    }
}

// void HumanoidSDK::MultiDynamixelWrite(uint8_t * dxl_id, uint8_t dxl_cnt, uint16_t addr, uint16_t * goal_positions)
// {
//     uint8_t dxl_error = 0;
//     int dxl_comm_result = COMM_TX_FAIL;

//     // Write data to Dynamixel
//     for (int i = 0; i < dxl_cnt; i++)
//     {
//         dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, dxl_id[i], addr, goal_positions[i], &dxl_error);
//         if (dxl_comm_result != COMM_SUCCESS) {
//             RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
//         } else if (dxl_error != 0) {
//             RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
//         }
//     }
// }

void HumanoidSDK::readDynamixel(uint8_t dxl_id, uint16_t addr)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;
    uint16_t present_position = 0;

    // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
    // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, addr, &present_position, &dxl_error);

    RCLCPP_INFO(this->get_logger(), "Get [ID: %d] [Present Position: %d]", dxl_id, present_position);
}