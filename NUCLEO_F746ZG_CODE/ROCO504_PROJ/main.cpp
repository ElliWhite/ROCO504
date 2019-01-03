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

/*=======
FUNCTIONS
=======*/   
void    preTensioning();
float   getGyroValues();
bool    POST();
void    findServos();

/*==========================
PRE-TENSION/FALLING BOOLEANS
==========================*/
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


/*==========
    MAIN    
==========*/
int main() {
    
    i2c.frequency(400000);          //use 400kHz I2C to MPU
    
    lMotorEN.period_ms(10);         //set motors' PWM period to 10ms
    rMotorEN.period_ms(10);
    
    bumperEnable = 1;                   //set bumper enable pin high
    lBumpTop.rise(&lMotorStop);         //attach left bumper top rising edge interrupt to left motor stop function
    lBumpBottom.rise(&lMotorStop);      //attach left bumper bottom rising edge interrupt to left motor stop function
    rBumpTop.rise(&rMotorStop);         //attach right bumper top rising edge interrupt to right motor stop function
    rBumpBottom.rise(&rMotorStop);      //attach right bumper bottom rising edge interrupt to right motor stop function
    lBumpTop.fall(&lMotorReleased);     //attach left bumper top falling edge interrupt to left motor released function
    lBumpBottom.fall(&lMotorReleased);  //attach left bumper bottom falling edge interrupt to left motor released function
    rBumpTop.fall(&rMotorReleased);     //attach right bumper top falling edge interrupt to right motor released function
    rBumpBottom.fall(&rMotorReleased);  //attach right bumper bottom falling edge interrupt to right motor released function
        
    
    lMotorBumped = false;
    rMotorBumped = false;
    
    //try and move right motor up for 1 second.
    //this is useful if the motor is stuck at the bottom..
    //..when robot is turned on
    rMotorUp(1.0f);
    motorTimeout.attach(&motorStop, 1.0);
       
    
    /*==
    POST
    ==*/
    //(power on self test)
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
    //create two instances of the Dynamixel class based on the two servo..
    //..ID's found in POST
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
    
    //need this delay as the status packet from RX28_1 torque toggle is.. 
    //..on the line so have to wait for that to finish
    wait_ms(1);
    
    /*===================
    RESET LEG RELEASE PIN
    ===================*/
    //move leg release pin back into locking position
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
    
    //try and calibrate right motor by moving it all the way to the bottom..
    //..and then bringing it back up slightly
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
        //vector to hold 50 values of acceleration in x axis
        std::vector<float> gyroVals;
        float accel_average = 0;
        float cummulative_accel = 0;
        
        taughtLeft = false;
        taughtRight = false;
        fallingLeft = false;
        fallingRight = false;
    
        redLED = 0;
        greenLED = 1;
        
        //remember the position of the leg before it falls
        uint16_t originalLegPos = RX28_1.getPosition();

        uint16_t newLegPos = originalLegPos;
        
        //pre-tension the leg based on which way it is being tipped
        preTensioning();
        
        //falling so turn on red LED
        redLED = 1;
        greenLED = 0;
        
        PCSERIAL("LEG FALLING\n");
        
        
        //deploy leg to right
        if(fallingRight == true){
            
            //pull up left motor to swing leg round
            lMotorUp(1.0f);
            //stop motor after 5 seconds
            motorTimeout.attach(&motorStop, 5.0f);
            //release leg pin
            RX28_2.move(700);
            
            //stay in loop unless acceleration of body has stabalised
            while(1){
                //measure the acceleration in x axis 50 times and save into vector
                for(int n=0; n<50; n++){
                    gyroVals.push_back(getGyroValues());
                    cummulative_accel = cummulative_accel + accel_x;
                    wait_ms(3);
                }
                
                //calculate average acceleration over the 50 samples
                accel_average = (cummulative_accel)/50;
                
                PCSERIAL("%f\n", accel_average);
                
                //if acceleration has stabalised to over 0.5 then we know body..
                //..has steadied. Refer to MATLAB graphs as to why we use 0.5..
                //..and not 0
                if(accel_average > 0.5f){
                    lMotorDown(0.4);        //start moving left motor down slowly
                    break;                  //break out of while loop
                }else{
                    gyroVals.clear();       //reset vector
                    cummulative_accel = 0;  //reset cummulative acceleration value
                }
            }
            
            //get position of the leg
            newLegPos = RX28_1.getPosition();
            
            //stay in loop until leg is within 20 degrees of being perpendicular..
            //..to the body. 3.4 units = 1 degree. While this is happening the left..
            //..motor is still moving down, or trying to.
            while(abs(newLegPos-originalLegPos)>68){
                newLegPos = RX28_1.getPosition();
                PCSERIAL("%d\n", newLegPos);
            }
            
            //stop motor once body is relatively flat
            motorTimeout.attach(&motorStop, 0.0);

            //turn on finished LED
            greenLED = 1;
            
        //deploy leg to left
        }else if(fallingLeft == true){ 
        
            //pull up right motor to swing leg round
            rMotorUp(1.0f);
            //stop motor after 5 seconds
            motorTimeout.attach(&motorStop, 5.0f);
            //release leg pin
            RX28_2.move(700);
            
            //stay in loop unless acceleration of body has stabalised
            while(1){
                //measure the acceleration in x axis 50 times and save into vector
                for(int n=0; n<50; n++){
                    gyroVals.push_back(getGyroValues());
                    cummulative_accel = cummulative_accel + accel_x;
                    wait_ms(3);
                }
                
                //calculate average acceleration over the 50 samples
                accel_average = (cummulative_accel)/50;
                
                PCSERIAL("%f\n", accel_average);
                
                //if acceleration has stabalised to over 0.5 then we know body..
                //..has steadied. Refer to MATLAB graphs as to why we use 0.5..
                //..and not 0
                if(accel_average > 0.5f){
                    rMotorDown(0.4);        //start moving right motor down slowly
                    break;                  //break out of while loop
                }else{
                    gyroVals.clear();       //reset vector
                    cummulative_accel = 0;  //reset cummulative acceleration value
                }
            }
            
            //get position of the leg
            newLegPos = RX28_1.getPosition();
            
            //stay in loop until leg is within 20 degrees of being perpendicular..
            //..to the body. 3.4 units = 1 degree. While this is happening the right..
            //..motor is still moving down, or trying to.
            while(abs(newLegPos-originalLegPos)>50){
                newLegPos = RX28_1.getPosition();
                PCSERIAL("%d\n", newLegPos);
            }
            
            //stop motor once body is relatively flat
            motorTimeout.attach(&motorStop, 0.0);

            //turn on finished LED
            greenLED = 1;
        
        }
        
        //exist with successful execution  
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
        //read accleration data
        mpu.readAccelData(accelCount);
        mpu.getAres();
        
        //calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes - accelBias[0];  //get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes - accelBias[1];
        az = (float)accelCount[2]*aRes - accelBias[2];
        
        //read gyro data
        mpu.readGyroData(gyroCount);
        mpu.getGres();
        
        //calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes;      //get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes;
        gz = (float)gyroCount[2]*gRes;

    }
    
    //pass gyro rate as rad/s
    mpu.MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);
    
    //convert Quaternions to Euler angles
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
        
        //booleans are used to make sure each if statement is only executed once
        
        //falling right if looking at robot from front
        if(yAngle >= 4 && taughtLeft == false){
            //tension left, release right
            //lMotorUp(1.0f);
            rMotorDown(0.7f);
            //move motors for 2 seconds
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
            //move motors for 2 seconds
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
    
    //toggle LEDs
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
        
    //find which Dynamixel servos are in the chain
    findServos();
    
    //print found servo ID numbers
    for(char i = 0; i < NumberOfServos; i++){
        PCSERIAL("SERVO ID = %d\n", ServoID[i]);
    }
        
    //create two instances of Dynamixel classes with the found servos
    Dynamixel _RX28_1(DynamixelTX, DynamixelRX, txEnable, ServoID[0], DynamixelBaud);
    Dynamixel _RX28_2(DynamixelTX, DynamixelRX, txEnable, ServoID[1], DynamixelBaud);

    //try to ping the servos and record the error (if there is one)
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
    
    
    //read 'who am I' register to test communication with MPU
    uint8_t whoami = mpu.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);
    PCSERIAL("TESTING COMMS\n");
    PCSERIAL("MPU ID = %x\n", whoami);
    
    //whoami should be equal to 0x68 which is hard-coded into the MPU
    if(whoami != 0x68) {
        PCSERIAL("MPU COMMS FAIL OR ID INCORRECT - EXITING\n");
        exit(EXIT_FAILURE);
    }
    
    PCSERIAL("MPU6050 ONLINE\n");
    
    //test the MPU
    mpu.MPU6050SelfTest(SelfTest);
    
    //if tests have returned values within the correct ranges then begin 
    //..setting up MPU
    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
        mpu.resetMPU6050();                         //reset
        mpu.calibrateMPU6050(gyroBias, accelBias);  //calibrate
        mpu.initMPU6050();                          //initialise
        PCSERIAL("MPU INITIALISED\n");
    } else {
        PCSERIAL("MPU FAILED TO PASS SELF-TEST\n");
    }
    
    //return result back to main
    return _POST_result;
    
}


/*================
    findServos      
================*/
void findServos(){
    
    yellowLED = 1;
    
    /*=========================================================================
    THE FOLLOWING BLOCK OF CODE WAS USED TO TRY AND SET ALL BAUD RATES OF 
    SERVOS IN THE CHAIN TO 57600. THIS WAS DONE AS THE BAUD RATES OF THE SERVOS
    WERE NOT NECCESSARILY THE SAME TO BEGIN WITH.
    =========================================================================*/
    /*
    static const int baudRatesArr[] = {9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000, 1000000};
    vector<int> baudRates (baudRatesArr, baudRatesArr + sizeof(baudRatesArr) / sizeof(baudRatesArr[0]));
    
    //loop through all possibilites of baud rates as baud rates of the servos..
    //..in the chain are unknown
    for(int i=0; i<9; i++){
        //Using broadcast ID so only need to send once
        Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, BROADCAST_ID, baudRateArr[i]);
        PCSERIAL("\nTrying to set baud rate to 57600 for all servos at baudrate %d\n", baudRateArr[i]);
        uint8_t RX28_x_Res = RX28_x.setBaud();          //set baud rate
    }
    */
    
    Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, BROADCAST_ID, DynamixelBaud);
    RX28_x.setReturnDelay();    // set return delay to 256*2uS
    
    wait_ms(1000);
    
    char ServosFound = 0;       
    
    for(uint8_t i = 1; i < 253; i++){
        
        // redeclared Dynamixel object with changing servo ID
        Dynamixel RX28_x(DynamixelTX, DynamixelRX, txEnable, i, DynamixelBaud);
        uint8_t RX28_x_Res = RX28_x.findServo();        //try and find servo in chain with that ID
        
        //check servo error returned
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


