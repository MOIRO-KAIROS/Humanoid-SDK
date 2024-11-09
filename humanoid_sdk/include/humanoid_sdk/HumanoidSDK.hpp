#ifndef HHUMANOID_SDK_HUMANOIDSDK_HPP_
#define HUMANOID_SDK_HUMANOIDSDK_HPP_

#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table address for AX-12A
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

#define TORQUE_DISABLE 0
#define TORQUE_ENABLE 1

// Protocol version
#define PROTOCOL_VERSION 1.0  // Default Protocol version of DYNAMIXEL AX-12A

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL AX-12A
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

class HumanoidSDK
{
    public:
        HumanoidSDK();
        ~HumanoidSDK();

        /* Setting Dynamixel Model */
        void setupDynamixel(uint8_t dxl_id);
        void MultiDynamixelSetup(uint8_t * dxl_id, uint8_t dxl_cnt);

        /* Control Dynamixel AX-12A */
        // Write data to Dynamixel
        void writeDynamixel(uint8_t dxl_id, uint16_t addr, uint16_t data); 
        void MultiDynamixelWrite(uint8_t * dxl_id, uint8_t dxl_cnt, uint16_t addr, uint16_t * data);

        // Read data from Dynamixel
        uint16_t readDynamixel(uint8_t dxl_id, uint16_t addr);

    private:
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;

        uint8_t dxl_error = 0;
        uint32_t goal_position = 0;
        int dxl_comm_result = COMM_TX_FAIL;
};

#endif // HHUMANOID_SDK_HUMANOIDSDK_HPP_