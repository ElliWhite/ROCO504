#include "Dynamixel.h"
#include "mbed.h"

// ========== Compile Options ==========
#define ENABLE_uSerial 0
// ========== DEBUG Console ============
#if ENABLE_uSerial
    Serial debug_uSerial(USBTX, USBRX);
    #define DEBUG(...) { debug_uSerial.printf(__VA_ARGS__); }
#else
    #define DEBUG(...)
#endif

BusOut myleds(LED1, LED2, LED3);

Timer t;

bool timeOut;

Dynamixel::Dynamixel(PinName tx, PinName rx, PinName txEnable, uint8_t motorID, int baudrate) :  m_link(tx, rx), m_txEnable(txEnable), m_motorID(motorID), m_baudrate(baudrate)
{
    m_link.baud(m_baudrate);
}

uint8_t Dynamixel::ping()
{
    DEBUG("\n ---Ping--- \n");
    uint8_t elements = 6;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH] = {0,0,0,0,0,0};
    
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;    // ID
        instructionBuffer[3] = 0x02;         // Length
        instructionBuffer[4] = PING;         // Instruction
        instructionBuffer[5] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4]) & 0xFF;   // Check sum

         m_txEnable = 1;     // Enable Tx / Disable Rx
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);
        }
       
        m_txEnable = 0;     // Disable Tx / Enable Rx
    } else {
        // Dynamixel not writeable
    }
  
    
    if ( m_motorID != BROADCAST_ID ) {
        int i = 0;
        while(m_link.readable() == 0){}
        while(i<STATUS_PACKET_LENGTH) {
            //DEBUG("READING STATUS PACKET\n");
            statusBuffer[i] = m_link.getc();    // Read status packet
            i++;
        }
    }
    
    // Check status packet content
    for (int i = 0; i < STATUS_PACKET_LENGTH; i++) {
        DEBUG("%x ", statusBuffer[i]);
    }
    wait_ms(10);     
    if(statusBuffer[0]==0){
        statusBuffer[4]=128;
    }
    return statusBuffer[4]; // Return error
}

uint8_t Dynamixel::findServo()
{
    DEBUG("\n ---Finding Servo %d--- \n", m_motorID);
    uint8_t elements = 6;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH] = {0,0,0,0,0,0};

    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;    // ID
        instructionBuffer[3] = 0x02;         // Length
        instructionBuffer[4] = PING;         // Instruction
        instructionBuffer[5] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4]) & 0xFF;   // Check sum

        m_txEnable = 1;     // Enable Tx / Disable Rx
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);
        }
       
        m_txEnable = 0;     // Disable Tx / Enable Rx

    } else {
        // Dynamixel not writeable
    }
    wait_us(1);

    if ( m_motorID != BROADCAST_ID ) {
        int i = 0;
        //while(m_link.readable() == 0){}
        for(int j = 0; j < 5000; j++){
            if(m_link.readable()){
                while(i<STATUS_PACKET_LENGTH) {
                    statusBuffer[i] = m_link.getc();    // Read status packet
                    i++;
                    if(statusBuffer[0]==0xff){
                        break;
                    }                 
                }
            }
        }
    }
    
    // Check status packet content
    for (int i = 0; i < STATUS_PACKET_LENGTH; i++) {
        DEBUG("%x ", statusBuffer[i]);
    }
    
    return statusBuffer[0]; // Return first byte
}

uint8_t Dynamixel::setBaud()
{
    DEBUG("\n ---Setting Baud--- \n");
    uint8_t elements = 8;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH] = {0,0,0,0,0,0};
    
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = BROADCAST_ID;    // ID
        instructionBuffer[3] = 0x04;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_BAUD;
        instructionBuffer[6] = 0x22;         // Sets baud rate to 57600
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;   // Check sum

        m_txEnable = 1;     // Enable Tx / Disable Rx
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);
        }
       
        m_txEnable = 0;     // Disable Tx / Enable Rx
    } else {
        // Dynamixel not writeable
    }
    wait_ms(50);            // fix this!!!
    
    return statusBuffer[0]; // Return first byte
}

uint8_t Dynamixel::setReturnDelay()
{
    DEBUG("\n ---Setting Return Delay--- \n");
    uint8_t elements = 8;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH] = {0,0,0,0,0,0};
    
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = BROADCAST_ID;    // ID
        instructionBuffer[3] = 0x04;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_RETURN_DELAY;
        instructionBuffer[6] = 0xFF;         // Sets return delay  to 256*2uS
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;   // Check sum

        m_txEnable = 1;     // Enable Tx / Disable Rx
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);
        }
       
        m_txEnable = 0;     // Disable Tx / Enable Rx
    } else {
        // Dynamixel not writeable
    }
    wait_ms(10);            // fix this!!!
    
    return statusBuffer[0]; // Return first byte
}

uint16_t Dynamixel::getPosition()
{
    DEBUG("\n ---Getting Position--- \n");
    uint8_t elements = 8;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH+2];
    
    timeOut = false;
    t.reset();
    
    if (m_link.writeable()) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;   // ID
        instructionBuffer[3] = 0x04;        // Length
        instructionBuffer[4] = READ_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_CURRENT_POSITION_L;        // Starting address (Present Position [L])
        instructionBuffer[6] = 0x02;        // Number of bytes to read
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;  // Check sum
        
        m_txEnable = 1;
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);

        }
        
        m_txEnable = 0;
    } else {
        // Dynamixel not writeable
    }
    
    wait_us(200);
    
    if ( m_motorID != BROADCAST_ID ) {
        int i = 0;
        t.start();
        while(m_link.readable() == 0){
            if(t.read() > 0.05f){
                timeOut = true;
                break;  
            }  
        }
        while(i<STATUS_PACKET_LENGTH+2 && timeOut == false) {
            //DEBUG("READING STATUS PACKET\n");
            statusBuffer[i] = m_link.getc();    // Read status packet
            i++; 
        }
        t.stop();
    }
    // Check status packet content
    for (int i = 0; i < STATUS_PACKET_LENGTH+2; i++) {
        DEBUG("%x ", statusBuffer[i]);
    }
    uint16_t upperPos = uint8_t(statusBuffer[6]) << 8;
    
    return upperPos+statusBuffer[5]; // Return servo position
    
}

void Dynamixel::torqueToggle(char enable)
{
    DEBUG("\n ---Turning off torque to motor %d--- \n", m_motorID);
    uint8_t elements = 8;
    uint8_t instructionBuffer[elements];
    
    if (m_link.writeable()) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;   // ID
        instructionBuffer[3] = 0x04;        // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_TORQUE_ENABLE;        // Torque enable address
        instructionBuffer[6] = enable;        // Enable/disable
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;  // Check sum
        
        m_txEnable = 1;
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);

        }
        
        m_txEnable = 0;
        
    } else {
        // Dynamixel not writeable
    }

    
}
    


uint8_t Dynamixel::toggleLED(uint8_t ledState)
{
    DEBUG("\n ---Toggle--- \n");
    uint8_t elements = 8;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH];
    
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;    // ID
        instructionBuffer[3] = 0x04;         // Length
        instructionBuffer[4] = WRITE_DATA;   // Instruction
        instructionBuffer[5] = ADDRESS_LED;  // Parameter 1: Starting address
        instructionBuffer[6] = ledState;     // Parameter 2: First value to be writen
        instructionBuffer[7] = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]) & 0xFF;   // Check sum

        m_txEnable = 1;     // Enable Tx / Disable Rx
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);
        }
       
        m_txEnable = 0;     // Disable Tx / Enable Rx
    } else {
        //Dynamixel not writeable
    }
    
    
    return statusBuffer[4]; // Return error
}

uint8_t Dynamixel::move(uint16_t position)
{
    DEBUG("\n ---Move--- \n");
    // 0 to 1023 (0x3FF)
    uint8_t elements = 9;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH] = {0,0,0,0,0,0};
    
    uint16_t checkSum = 0;
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;                    // ID
        instructionBuffer[3] = 0x05;                         // Length
        instructionBuffer[4] = WRITE_DATA;                   // Instruction
        instructionBuffer[5] = ADDRESS_GOAL_POSITION;        // Parameter 1: Starting address
        instructionBuffer[6] = (uint8_t) position & 0xff;    // Parameter 2: First value to be writen
        instructionBuffer[7] = (uint8_t) (position >> 8);    // Parameter 3: Second value to be writen
        checkSum = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]);
        instructionBuffer[8] = (uint8_t) (checkSum & 0xff); // Check sum
        // Check instruction buffer
        for (int i = 0; i <  elements; i++) {
            DEBUG("%x ", instructionBuffer[i]);
        }
        DEBUG("\n");
        
        m_txEnable = 1;     // Enable Tx / Disable Rx
        
        wait_ms(1);     // need this wait so the transmitter start-up
        
        for (int i = 0; i < (elements + 2); i++) {
            m_link.putc(instructionBuffer[i]);
        }
       
        m_txEnable = 0;     // Disable Tx / Enable Rx
    } else {
        //Dynamixel not writeable
    }
    
    
    
    return statusBuffer[4]; // Return error
}

uint8_t Dynamixel::setSpeed(uint16_t speed)
{
    DEBUG("\n --- Set speed--- \n");
    // 0 to 1023 (0x3FF)
    uint8_t elements = 9;
    uint8_t instructionBuffer[elements];
    uint16_t checkSum = 0;
    uint8_t statusBuffer[STATUS_PACKET_LENGTH];
    
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;                    // ID
        instructionBuffer[3] = 0x05;                         // Length
        instructionBuffer[4] = WRITE_DATA;                   // Instruction
        instructionBuffer[5] = ADDRESS_MOVING_SPEED;         // Parameter 1: Starting address
        instructionBuffer[6] = (uint8_t) speed & 0xff;       // Parameter 2: First value to be writen
        instructionBuffer[7] = (uint8_t) (speed >> 8);       // Parameter 3: Second value to be writen
        checkSum = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]);
        instructionBuffer[8] = (uint8_t) (checkSum & 0xff); // Check sum

        m_txEnable = 1;       // Enable Tx / Disable Rx
        for (int i = 0; i<= elements; i++) {
            m_link.putc(instructionBuffer[i]);
        }
        wait_ms(2);         // fix this!!!
        m_txEnable = 0;       // Disable Tx / Enable Rx
    } else {
        //Dynamixel not writeable
    }
    wait_ms(10);
    if ( m_motorID != BROADCAST_ID ) {
        DEBUG("Trying to receiv status packet.\n");
        int i = 0;
        while( m_link.readable() && i<STATUS_PACKET_LENGTH) {
            statusBuffer[i] = m_link.getc();    // Read status packet
            i++;
            //wait_ms(100);
        }
        DEBUG("Read %d bytes.\n", i);
    }
    // Check status packet content
    for (int i = 0; i < STATUS_PACKET_LENGTH; i++) {
        DEBUG("%x ", statusBuffer[i]);
    }
    return statusBuffer[4]; // Return error
}

uint8_t Dynamixel::getReturnDelayTime()
{
    // 0 to 254 (0xFE)
    uint8_t elements = 8;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH+1] = {0,0,0,0,0,0,0}; // usual length plus the value read

    uint16_t checkSum = 0;
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;                   // ID
        instructionBuffer[3] = 0x04;                        // Length
        instructionBuffer[4] = READ_DATA;                   // Instruction
        instructionBuffer[5] = ADDRESS_RETURN_DELAY_TIME;   // Parameter 1: Starting address
        instructionBuffer[6] = 0x01;                        // Parameter 2: Number of bytes to read
        
        checkSum = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6]);
        instructionBuffer[7] = (uint8_t) (checkSum & 0xff); // Check sum
        // Check instruction buffer
        for (int i = 0; i <  elements; i++) {
            DEBUG("%x ", instructionBuffer[i]);
        }
        DEBUG("\n");

        m_txEnable = 1;       // Enable Tx / Disable Rx
        for (int i = 0; i<= elements; i++) {
            m_link.putc(instructionBuffer[i]);
        }
        wait_ms(2);         // fix this!!!
        m_txEnable = 0;       // Disable Tx / Enable Rx
    } else {
        //Dynamixel not writeable
    }

    wait_ms(10);
    if ( m_motorID != BROADCAST_ID ) {
        DEBUG("Trying to receiv status packet.\n");
        int i = 0;
        while( m_link.readable() && i<STATUS_PACKET_LENGTH + 1) {
            statusBuffer[i] = m_link.getc();    // Read status packet
            i++;
            //wait_ms(100);
        }
        DEBUG("Read %d bytes.\n", i);
    }
    // Check status packet content
    for (int i = 0; i < STATUS_PACKET_LENGTH + 1; i++) {
        DEBUG("%x ", statusBuffer[i]);
    }
    DEBUG("\n Delay time: %d [us]\n",statusBuffer[5]*2);

    return statusBuffer[5]; // Return parameter
}

uint8_t Dynamixel::setAngleLimit(uint16_t mode, uint8_t address)
{
    DEBUG("\n ---setAngleLimit--- \n");
    // Sets allowable position values (angles) for Goal position
    /* 
    Valid address
    -------------
    Use the values defined in the header file:
     - ADDRESS_CW_ANGLE_LIMIT
     - ADDRESS_CCW_ANGLE_LIMIT
    Mode values
    -----------
     - Wheel mode: set both CW and CCW registers to 0
     - Joint mode: set both CW and CCW registers to a value different of 0
     - Multi-turn mode: both CW and CCW registers are 4095 (0x0FFF)
    */
    uint8_t elements = 9;
    uint8_t instructionBuffer[elements];
    uint8_t statusBuffer[STATUS_PACKET_LENGTH] = {0,0,0,0,0,0};
    
    uint16_t checkSum = 0;
    if (m_link.writeable() ) {
        instructionBuffer[0] = 0xff;
        instructionBuffer[1] = 0xff;
        instructionBuffer[2] = m_motorID;                   // ID
        instructionBuffer[3] = 0x05;                        // Length
        instructionBuffer[4] = WRITE_DATA;                  // Instruction
        instructionBuffer[5] = address;                     // Parameter 1: Starting address (Low byte)
        instructionBuffer[6] = (uint8_t) mode & 0xff;       // Parameter 2: First value to be writen
        instructionBuffer[7] = (uint8_t) (mode >> 8);       // Parameter 3: Second value to be writen
        checkSum = ~(instructionBuffer[2] + instructionBuffer[3] + instructionBuffer[4] + instructionBuffer[5] + instructionBuffer[6] + instructionBuffer[7]);
        instructionBuffer[8] = (uint8_t) (checkSum & 0xff); // Check sum
        // Check instruction buffer
        for (int i = 0; i <  elements; i++) {
            DEBUG("%x ", instructionBuffer[i]);
        }
        DEBUG("\n");
        
        m_txEnable = 1;       // Enable Tx / Disable Rx
        for (int i = 0; i<= elements; i++) {
            m_link.putc(instructionBuffer[i]);
        }
        wait_ms(2);         // fix this!!!
        m_txEnable = 0;       // Disable Tx / Enable Rx
    } else {
        //Dynamixel not writeable
    }
    
    wait_ms(10);
    if ( m_motorID != BROADCAST_ID ) {
        DEBUG("Trying to receiv status packet.\n");
        int i = 0;
        while( m_link.readable() && i<STATUS_PACKET_LENGTH) {
            statusBuffer[i] = m_link.getc();    // Read status packet
            i++;
            //wait_ms(100);
        }
        DEBUG("Read %d bytes.\n", i);
    }
    // Check status packet content
    for (int i = 0; i < STATUS_PACKET_LENGTH; i++) {
        DEBUG("%x ", statusBuffer[i]);
    }
    
    return statusBuffer[4]; // Return error
}