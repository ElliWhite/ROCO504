#ifndef DEF_DYNAMIXEL
#define DEF_DYNAMIXEL

#include "mbed.h"

const unsigned char BROADCAST_ID = 0xfe;
const unsigned char STATUS_PACKET_LENGTH = 0x06;
// Instruction set
const unsigned char PING = 0x01;
const unsigned char READ_DATA = 0x02;
const unsigned char WRITE_DATA = 0x03;
const unsigned char REG_WRITE = 0x04;
const unsigned char ACTION = 0x05;
const unsigned char dRESET = 0x06;
const unsigned char SYNC_WRITE = 0x83;
// Control table
const unsigned char ADDRESS_PRESENT_TEMPERATURE = 0x2B;
const unsigned char ADDRESS_GOAL_POSITION = 0x1E;
const unsigned char ADDRESS_LED = 0x19;
const unsigned char ADDRESS_MOVING_SPEED = 0x20;
const unsigned char ADDRESS_RETURN_DELAY_TIME = 0x05;
const unsigned char ADDRESS_MAX_TORQUE = 0x0E;
const unsigned char ADDRESS_TORQUE_ENABLE = 0x18;
const unsigned char ADDRESS_CW_ANGLE_LIMIT = 0x06;
const unsigned char ADDRESS_CCW_ANGLE_LIMIT = 0x08;
const unsigned char ADDRESS_BAUD = 0x04;
const unsigned char ADDRESS_RETURN_DELAY = 0x05;
const unsigned char ADDRESS_CURRENT_POSITION_L = 0x24;

const unsigned char TORQUE_ENABLE = 0x01;
const unsigned char TORQUE_DISABLE = 0x00;

class Dynamixel
{
public:
    Dynamixel(PinName tx, PinName rx, PinName txEnable, uint8_t motorID, int baudrate);
    uint8_t ping();
    uint8_t toggleLED(uint8_t ledState);
    uint8_t move(uint16_t position);
    uint8_t setAngleLimit(uint16_t mode, uint8_t address);
    uint8_t setSpeed(uint16_t speed);
    uint8_t setReturnDelayTime(uint16_t speed);
    uint8_t getReturnDelayTime();
    uint8_t findServo();
    uint8_t setBaud();
    uint16_t getPosition();
    uint8_t setReturnDelay();
    void torqueToggle(char enable);     // no status packet returned
private:
    Serial m_link;
    DigitalOut m_txEnable;
    uint8_t m_motorID;
    const int m_baudrate;
};
#endif
