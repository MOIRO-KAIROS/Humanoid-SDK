#ifndef HHUMANOID_SDK_HUMANOIDSDK_HPP_
#define HUMANOID_SDK_HUMANOIDSDK_HPP_

#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table address for AX-12A
#define ADDR_TORQUE_ENABLE 24
#define ADDR_GOAL_POSITION 30
#define ADDR_PRESENT_POSITION 36

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

    private:
        dynamixel::PortHandler * portHandler;
        dynamixel::PacketHandler * packetHandler;

        uint8_t dxl_error = 0;
        uint32_t goal_position = 0;
        int dxl_comm_result = COMM_TX_FAIL;
};

#endif // HHUMANOID_SDK_HUMANOIDSDK_HPP_