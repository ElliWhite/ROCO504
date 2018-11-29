#include "mbed.h"
#include "Dynamixel.h"
#include "mbed.h"
#include <vector>
#include "MPU6050.h"


/*=============
PIN DEFINITIONS
=============*/
#define DynamixelTX     PD_5
#define DynamixelRX     PD_6
#define txEnable        PC_10
#define DebugLED1       PC_11
#define DebugLED2       PC_12
#define DebugLED3       PD_2
#define lMotorIN1Pin    PG_0
#define lMotorIN2Pin    PD_1
#define rMotorIN3Pin    PD_0
#define rMotorIN4Pin    PF_8
#define lMotorENPin     PF_9
#define rMotorENPin     PF_7


/*============
DYNAMIXEL INFO
============*/
#define NumberOfServos 2
#define DynamixelBaud 57600
uint8_t ServoID[NumberOfServos];


/*====
SERIAL
====*/
Serial PC_serial(USBTX, USBRX);
#define PCSERIAL(...) { PC_serial.printf(__VA_ARGS__); }


/*=
MPU
=*/
MPU6050 mpu;
float xAngle = 0;
float yAngle = 0;
float zAngle = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;


/*==
LEDS
==*/
DigitalOut redLED(DebugLED1);
DigitalOut yellowLED(DebugLED2);
DigitalOut greenLED(DebugLED3);


/*==============
DC MOTOR CONTROL
==============*/
DigitalOut  lMotorIN1(lMotorIN1Pin);
DigitalOut  lMotorIN2(lMotorIN2Pin);
PwmOut      lMotorEN(lMotorENPin);

DigitalOut  rMotorIN3(rMotorIN3Pin);
DigitalOut  rMotorIN4(rMotorIN4Pin);
PwmOut      rMotorEN(rMotorENPin);

Ticker motorTimer;

void motorStop(){
    lMotorEN.write(0.0f);
    rMotorEN.write(0.0f);
    motorTimer.detach();
}


#define POST_SUCCESS true
#define POST_FAIL false


bool    POST();
void    findServos();
float   getGyroValues();
void    preTensioning();

bool    taughtLeft = false;
bool    taughtRight = false;
bool    fallingLeft = false;
bool    fallingRight = false;


/*==========
    MAIN    
==========*/
int main() {
    
    i2c.frequency(400000);  //use 400kHz I2C to MPU
    
    lMotorEN.period_ms(10);
    rMotorEN.period_ms(10);

    /*==
    POST
    ==*/
    bool POST_result = POST();

    if(POST_result == POST_FAIL){
        //quit program as POST has failed
        PCSERIAL("*********************\n");
        PCSERIAL("POST FAILED - EXITING\n");
        PCSERIAL("*********************\n");
        yellowLED = 1;
        exit(EXIT_FAILURE);
    }
    
    /*======================
    Dynamixel Class Creation
    ======================*/
    Dynamixel RX28_1(DynamixelTX, DynamixelRX, txEnable, ServoID[0], DynamixelBaud);
    Dynamixel RX28_2(DynamixelTX, DynamixelRX, txEnable, ServoID[1], DynamixelBaud);

    /*=================
    TOGGLING SERVO LEDS
    =================*/
    for(uint8_t i = 0; i < 3; i++){
        PCSERIAL("TOGGLING SERVO 1\n");
        RX28_1.toggleLED(0);
        wait_ms(75);
        RX28_1.toggleLED(1);
        wait_ms(75);
    }
    for(uint8_t i = 0; i < 3; i++){
        PCSERIAL("TOGGLING SERVO 2\n");
        RX28_2.toggleLED(0);
        wait_ms(75);
        RX28_2.toggleLED(1);
        wait_ms(75);
    }
    
    /*=============
    TURN OFF TORQUE
    =============*/
    RX28_1.torqueToggle(TORQUE_DISABLE);
    
    deltat = 0.01875;
    
    //need this delay as the status packet from RX28_1 torque toggle is 
    //on the line so have to wait for that to finish
    wait_ms(1);
    
    RX28_2.move(512);
    
    PCSERIAL("Starting up MPU\n");
    
    //allow gyro to start up and read true value
    for(int i = 0; i < 5000; i++){
        getGyroValues();
    }
        
    getGyroValues();
    
    PCSERIAL("**************************************\n\n");
    PCSERIAL("ROBOT MUST BE VERTICAL DURING START-UP\n");
    PCSERIAL("**************************************\n\n");
    
    //wait for robot to be lifted up
    while(yAngle > 2 || yAngle < -2){
        getGyroValues();
        redLED = 1;
    }
    
    PCSERIAL("Robot Vertical\n");
    
    //allow robot to be steadied so doesn't detect a fall straight away
    wait_ms(1000);
    
    while(1){
        
        taughtLeft = false;
        taughtRight = false;
        fallingLeft = false;
        fallingRight = false;
    
        redLED = 0;
        greenLED = 1;
        
        preTensioning();
        
        //falling
        redLED = 1;
        greenLED = 0;
        
        //need to pull leg to a certain point e.g 90 degrees in the right direction
        uint16_t oldLegPos = RX28_1.getPosition();
        
        //change of around 300 in position ~90 degrees
        uint16_t newLegPos = RX28_1.getPosition();
        
        //release leg pin
        RX28_2.move(1023);
        
        PCSERIAL("LEG FALLING\n");
        
        //deploy leg to right
        if(fallingRight == true){
            lMotorIN1 = 1;
            lMotorIN2 = 0;
            rMotorIN3 = 0;
            rMotorIN4 = 1;
            lMotorEN.write(1.0f);
            rMotorEN.write(1.0f);
            
            //keep moving until leg has moved 90 degrees (abs keeps number positive)
            while((abs(newLegPos - oldLegPos)) < 300){
                //maybe loop until leg has been deployed
                newLegPos = RX28_1.getPosition();   
            }
            motorTimer.attach(&motorStop, 0.5);
            
        //deploy leg to left
        }else if(fallingLeft == true){  
            lMotorIN1 = 0;
            lMotorIN2 = 1;
            rMotorIN3 = 1;
            rMotorIN4 = 0;
            lMotorEN.write(1.0f);
            rMotorEN.write(1.0f);
            
            //keep moving until leg has moved 90 degrees (abs keeps number positive)
            while((abs(newLegPos - oldLegPos)) < 300){
                //maybe loop until leg has been deployed
                newLegPos = RX28_1.getPosition();   
            }
            motorTimer.attach(&motorStop, 0.5);            
        }
        
        
        
        float old_accel_val = 0;
        
        getGyroValues();
        
        std::vector<float> old_accel_list;
        std::vector<float> accel_list;
        
        /*
        Timer t;
        t.start();
        float timePassed = t.read();
        
        while(timePassed < 2.0f){
                
            //measure acceleration and angle of robot. Save in old_accel_val
            old_accel_list.push_back(getGyroValues());
            //old_accel_val = getGyroValues();
            wait_ms(3);
            //read again to get new acceleration value
            getGyroValues();
            accel_list.push_back(accel_x);
            //PCSERIAL("old_accel_val = %f \t accel_x = %f\n", old_accel_val, accel_x); 
            timePassed = t.read();
            wait_ms(3);
                
        }
        
        PCSERIAL("OLD_ACCEL_LIST\n");
        
        for(int i = 0; i<old_accel_list.size(); i++){
            PCSERIAL("%f\n", old_accel_list[i]);
        }
        
        PCSERIAL("ACCEL_LIST\n");
        
        for(int i = 0; i<accel_list.size(); i++){
            PCSERIAL("%f\n", accel_list[i]);
        }
           
        */
        //keep measuring acceleration until impact
        while(old_accel_val - accel_x < 0.4f){
            //measure acceleration and angle of robot. Save in old_accel_val
            old_accel_val = getGyroValues();
            wait_ms(3);
            //read again to get new acceleration value
            getGyroValues();
            wait_ms(3);
            
        }
        
        PCSERIAL("HIT\n");
        redLED = 0;
        yellowLED = 1;
        

        
        //pull back leg to right
        if(fallingRight == true){
            lMotorIN1 = 0;
            lMotorIN2 = 1;
            rMotorIN3 = 1;
            rMotorIN4 = 0;
            lMotorEN.write(1.0f);
            rMotorEN.write(1.0f);
            motorTimer.attach(&motorStop, 0.5);

            
        //pull back leg to left
        }else if(fallingLeft == true){
            lMotorIN1 = 1;
            lMotorIN2 = 0;
            rMotorIN3 = 0;
            rMotorIN4 = 1;
            lMotorEN.write(1.0f);
            rMotorEN.write(1.0f);
            motorTimer.attach(&motorStop, 0.5);           
        }
        
        exit(EXIT_SUCCESS);
    }
    
  
}
/*==============
    END MAIN    
==============*/


/*==========
    POST    
==========*/
bool POST() {
 
    bool _POST_result;
    redLED = 0;
    yellowLED = 0;
    greenLED = 0;
    wait_ms(200);
    redLED = 1;
    wait_ms(200);
    yellowLED = 1;
    wait_ms(200);
    greenLED = 1;
    wait_ms(200);
    redLED = 0;
    yellowLED = 0;
    greenLED = 0;
    wait_ms(200);
        
    findServos();
    
    for(char i = 0; i < NumberOfServos; i++){
        PCSERIAL("SERVO ID = %d\n", ServoID[i]);
    }
        
    Dynamixel _RX28_1(DynamixelTX, DynamixelRX, txEnable, ServoID[0], DynamixelBaud);
    Dynamixel _RX28_2(DynamixelTX, DynamixelRX, txEnable, ServoID[1], DynamixelBaud);

    uint8_t RX28_1_Error = _RX28_1.ping();
    uint8_t RX28_2_Error = _RX28_2.ping();
    
    if(RX28_1_Error==0&&RX28_2_Error==0){
        //No errors
        PCSERIAL("\n============================\n");
        PCSERIAL("ALL SERVOS PINGED. NO ERRORS\n");
        PCSERIAL("============================\n");
        _POST_result = POST_SUCCESS; 
    }else{ 
        PCSERIAL("\n******************\n");
        PCSERIAL("SERVO ERRORS FOUND\n");
        PCSERIAL("******************\n");
        _POST_result = POST_FAIL;
    }
    
    if(RX28_1_Error == 128){
        PCSERIAL("******************\n");
        PCSERIAL("RX28_1 UNREACHABLE\n");
        PCSERIAL("******************\n");        
    }else if(RX28_1_Error != 0){
        PCSERIAL("*******************\n");
        PCSERIAL("RX28_1 ERROR = %x\n", RX28_2_Error);
        PCSERIAL("*******************\n");
    }
    if(RX28_2_Error == 128){
        PCSERIAL("******************\n");
        PCSERIAL("RX28_2 UNREACHABLE\n");
        PCSERIAL("******************\n");        
    }else if(RX28_2_Error != 0){
        PCSERIAL("*******************\n");
        PCSERIAL("RX28_2 ERROR = %x\n", RX28_2_Error);
        PCSERIAL("*******************\n");
    }
    
    
    // read 'who am I' register to test communication with MPU
    uint8_t whoami = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
    PCSERIAL("TESTING COMMS\n");
    PCSERIAL("MPU ID = %x\n", whoami);
    
    if(whoami != 0x68) {
        PCSERIAL("MPU COMMS FAIL OR ID INCORRECT - EXITING\n");
        exit(EXIT_FAILURE);
    }
    
    PCSERIAL("MPU6050 ONLINE\n");
    
    mpu.MPU6050SelfTest(SelfTest);
    
    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
        mpu.resetMPU6050();
        mpu.calibrateMPU6050(gyroBias, accelBias);
        mpu.initMPU6050();
        PCSERIAL("MPU INITIALISED\n");
    } else {
        PCSERIAL("MPU FAILED TO PASS SELF-TEST\n");
    }
    
        
    return _POST_result;
    
}


/*================
    findServos      
================*/
void findServos(){
    
    yellowLED = 1;
    
    //static const int baudRatesArr[] = {9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000};
    //vector<int> baudRates (baudRatesArr, baudRatesArr + sizeof(baudRatesArr) / sizeof(baudRatesArr[0]));
 
    
    /*============================================
    ALL 4 SERVOS IN CHAIN ARE ALREADY SET TO 57600
    ============================================*/
    
    //Using broadcast ID so only need to send once
    //Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, BROADCAST_ID, 9600);
    //PCSERIAL("\nTrying to set baud rate to 57600 for all servos at baudrate %d\n", 9600);
    //uint8_t RX28_x_Res = RX28_x.setBaud();
    
    //Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, BROADCAST_ID, DynamixelBaud);
    //RX28_x.setReturnDelay();    // set return delay to 256*2uS
        
    
    
    //wait_ms(2000);
    
    char ServosFound = 0;
    
    /*
    while(1){
        Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, 1, DynamixelBaud);
        uint8_t RX28_x_Res = RX28_x.findServo();
    }
    */
        
    
    for(uint8_t i = 1; i < 253; i++){
        
        // redeclared Dynamixel object with changing servo ID
        Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, i, DynamixelBaud);
        uint8_t RX28_x_Res = RX28_x.findServo();
                
        if(RX28_x_Res == 0xff){
            PCSERIAL("SERVO ID %d UNREACHABLE\n", i);
        }else if(RX28_x_Res != 0xff){
            PCSERIAL("SERVO ID %d FOUND\n", i);
          
            switch (ServosFound) {
                case 0:
                    ServoID[0] = i;
                    break;
                case 1:
                    ServoID[1] = i;
                    break;
                default:
                    PCSERIAL("TOO MANY SERVOS FOUND\n");
            }
            ServosFound += 1;
        }
        
        if(ServosFound == NumberOfServos){
            PCSERIAL("\n================\n");
            PCSERIAL("ALL SERVOS FOUND\n");
            PCSERIAL("================\n");
            wait_ms(200);
            break;
        }
        
    }
    
    yellowLED = 0;
    
}


/*===================
    getGyroValues            
===================*/
float getGyroValues(){
    // check to see if 'data ready' bit set
    if(mpu.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
        
        mpu.readAccelData(accelCount);
        mpu.getAres();
        
        ax = (float)accelCount[0]*aRes - accelBias[0];
        ay = (float)accelCount[1]*aRes - accelBias[1];
        az = (float)accelCount[2]*aRes - accelBias[2];
        
        mpu.readGyroData(gyroCount);
        mpu.getGres();
        
        gx = (float)gyroCount[0]*gRes;
        gy = (float)gyroCount[1]*gRes;
        gz = (float)gyroCount[2]*gRes;

    }
    
    mpu.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);
    
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI; 
    roll  *= 180.0f / PI;
    
    xAngle = roll;
    yAngle = pitch;
    zAngle = yaw;
    
    accel_x = ax;
    accel_y = ay;
    accel_z = az;
    
    return accel_x;

}


/*===================
    preTensioning            
===================*/
void preTensioning(){
    //pre-tensioning while robot is standing.
    //any more than 20 degrees tilt is a 'fall'
    while(yAngle > -20 & yAngle < 20){
        getGyroValues();
        
        //falling right if looking at robot from front
        if(yAngle >= 5 && taughtLeft == false){
            //tension left, release right
            lMotorIN1 = 1;
            lMotorIN2 = 0;
            rMotorIN3 = 0;
            rMotorIN4 = 1;
            lMotorEN.write(1.0f);
            rMotorEN.write(1.0f);
            motorTimer.attach(&motorStop, 0.05);
            taughtLeft = true;
            taughtRight = false;
            fallingRight = true;
            fallingLeft = false;
        }
        
        //falling left if looking at robot from front
        if(yAngle <= -5 && taughtRight == false){
            //tension right, release left
            lMotorIN1 = 0;
            lMotorIN2 = 1;
            rMotorIN3 = 1;
            rMotorIN4 = 0;
            lMotorEN.write(1.0f);
            rMotorEN.write(1.0f);
            motorTimer.attach(&motorStop, 0.05);
            taughtLeft = false;
            taughtRight = true;
            fallingRight = false;
            fallingLeft = true;
        }

    }
    
    
}


