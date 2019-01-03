#include <list>
#include <vector>
#include "mbed.h"
#include "Dynamixel.h"
#include "MPU6050.h"
#include "motors.h"
#include "serial.h"


#define POST_SUCCESS    true
#define POST_FAIL       false

#define DebugLED1       PC_11
#define DebugLED2       PC_12
#define DebugLED3       PD_2



/*==
LEDS
==*/
DigitalOut redLED(DebugLED1);
DigitalOut yellowLED(DebugLED2);
DigitalOut greenLED(DebugLED3);

   
void    preTensioning();
float   getGyroValues();
bool    POST();
void    findServos();

bool    taughtLeft = false;
bool    taughtRight = false;
bool    fallingLeft = false;
bool    fallingRight = false;

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

//float SelfTest[6];
//float accelBias_2[3] = {0, 0, 0};


/*==========
    MAIN    
==========*/
int main() {
    
    i2c.frequency(400000);  //use 400kHz I2C to MPU
    
    lMotorEN.period_ms(10);
    rMotorEN.period_ms(10);
    
    bumperEnable = 1;
    lBumpTop.rise(&lMotorStop);
    lBumpBottom.rise(&lMotorStop);
    rBumpTop.rise(&rMotorStop);
    rBumpBottom.rise(&rMotorStop);
    lBumpTop.fall(&lMotorReleased);
    lBumpBottom.fall(&lMotorReleased);
    rBumpTop.fall(&rMotorReleased);
    rBumpBottom.fall(&rMotorReleased);
        
    
    lMotorBumped = false;
    rMotorBumped = false;
    
    
    rMotorUp(1.0f);
    motorTimeout.attach(&motorStop, 1.0);
       
    
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
    
    //allow gyro to start up and empty its FIFO
    for(int i = 0; i < 1024; i++){
        getGyroValues();
    }
    
    
    PCSERIAL("**************************************\n\n");
    PCSERIAL("ROBOT MUST BE VERTICAL DURING START-UP\n\n");
    PCSERIAL("**************************************\n\n");
    
    //wait for robot to be lifted up
    while(yAngle > 2 || yAngle < -2){
        getGyroValues();
        redLED = 1;
    }
    
    PCSERIAL("Robot Vertical\n\r");
    
    //allow robot to be steadied so doesn't detect a fall straight away
    wait_ms(1000);
    
    PCSERIAL("Calibratiing Right Motor\n\r");
    
    //try and calibrate motor
    rMotorDown(0.5f);
    while(1){
       if(!rMotorBumped){
           yellowLED = 1;  
       }else{
           yellowLED = 0;
           wait_ms(500);
           rMotorUp(1.0f);
           motorTimeout.attach(&motorStop, 4.0f);
           break;
       }
    }
    
    wait_ms(5000);
    
    PCSERIAL("Right Motor Calibrated\n\r");      
    
    while(1){
        
        std::vector<float> gyroVals;
        float accel_average = 0;
        float cummulative_accel = 0;
        
        taughtLeft = false;
        taughtRight = false;
        fallingLeft = false;
        fallingRight = false;
    
        redLED = 0;
        greenLED = 1;
        
        uint16_t originalLegPos = RX28_1.getPosition();

        uint16_t oldLegPos = originalLegPos;

        uint16_t newLegPos = originalLegPos;
        
        preTensioning();
        
        //falling
        redLED = 1;
        greenLED = 0;
        
        PCSERIAL("LEG FALLING\n");
        
        
        
        //deploy leg to right
        if(fallingRight == true){
            
            lMotorUp(1.0f);
            motorTimeout.attach(&motorStop, 5.0f);
            //release leg pin
            RX28_2.move(700);
            
            while(1){
                for(int n=0; n<50; n++){
                    gyroVals.push_back(getGyroValues());
                    cummulative_accel = cummulative_accel + accel_x;
                    accel_average = (cummulative_accel)/(n+1);
                    wait_ms(3);
                }
                
                PCSERIAL("%f\n", accel_average);
                
                if(accel_average > 0.5f){
                    lMotorDown(0.4);
                    break;
                }else{
                    gyroVals.clear();
                    cummulative_accel = 0;
                }
            }
            
            newLegPos = RX28_1.getPosition();
            
            while(abs(newLegPos-originalLegPos)>50){
                newLegPos = RX28_1.getPosition();
                PCSERIAL("%d\n", newLegPos);
            }
            
            motorTimeout.attach(&motorStop, 0.0);

            greenLED = 1;
            
        //deploy leg to left
        }else if(fallingLeft == true){ 
        
            rMotorUp(1.0f);
            motorTimeout.attach(&motorStop, 5.0f);
            //release leg pin
            RX28_2.move(700);
            
            while(1){
                for(int n=0; n<50; n++){
                    gyroVals.push_back(getGyroValues());
                    cummulative_accel = cummulative_accel + accel_x;
                    accel_average = (cummulative_accel)/(n+1);
                    wait_ms(3);
                }
                
                PCSERIAL("%f\n", accel_average);
                
                if(accel_average > 0.5f){
                    rMotorDown(0.4);
                    break;
                }else{
                    gyroVals.clear();
                    cummulative_accel = 0;
                }
            }
            
            newLegPos = RX28_1.getPosition();
            
            while(abs(newLegPos-originalLegPos)>50){
                newLegPos = RX28_1.getPosition();
                PCSERIAL("%d\n", newLegPos);
            }
            
            motorTimeout.attach(&motorStop, 0.0);

            greenLED = 1;
   
     
                     
        }
        

        /*   
        *****************************
        THIS ONLY WORKS FOR WHEN 
        THE BODY HAS HIT, NOT THE LEG
        *****************************
        
        //keep measuring acceleration until impact
        while(old_accel_val - accel_y < 0.4f){
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
        
        motorTimeout.attach(&motorStop, 0.00);
        */
        
        
        
        exit(EXIT_SUCCESS);
    }
    
  
}
/*==============
    END MAIN    
==============*/





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
    //any more than 10 degrees tilt is a 'fall'
    while(yAngle > -10 & yAngle < 10){
        getGyroValues();
        
        //falling right if looking at robot from front
        if(yAngle >= 4 && taughtLeft == false){
            //tension left, release right
            //lMotorUp(1.0f);
            rMotorDown(0.7f);
            motorTimeout.attach(&motorStop, 2.0);
            taughtLeft = true;
            taughtRight = false;
            fallingRight = true;
            fallingLeft = false;
        }
        
        //falling left if looking at robot from front
        if(yAngle <= -4 && taughtRight == false){
            //tension right, release left
            //lMotorDown(1.0f);
            rMotorUp(1.0f);
            motorTimeout.attach(&motorStop, 2.0);
            taughtLeft = false;
            taughtRight = true;
            fallingRight = false;
            fallingLeft = true;
        }

    }
    
    
}


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
        PCSERIAL("RX28_1 ERROR = %x\n", RX28_1_Error);
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


