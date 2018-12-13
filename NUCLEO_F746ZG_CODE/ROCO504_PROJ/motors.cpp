#include "mbed.h"
#include "motors.h"

#define lMotorIN1Pin    PG_0
#define lMotorIN2Pin    PD_1
#define rMotorIN3Pin    PD_0
#define rMotorIN4Pin    PF_8
#define lMotorENPin     PF_9
#define rMotorENPin     PF_7
#define lBumpTopPin     PF_12   //purple
#define lBumpBottomPin  PD_14   //yellow
#define rBumpTopPin     PA_7    //green
#define rBumpBottomPin  PD_15   //blue

/*==============
DC MOTOR CONTROL
==============*/
DigitalOut  lMotorIN1(lMotorIN1Pin);
DigitalOut  lMotorIN2(lMotorIN2Pin);
PwmOut      lMotorEN(lMotorENPin);

DigitalOut  rMotorIN3(rMotorIN3Pin);
DigitalOut  rMotorIN4(rMotorIN4Pin);
PwmOut      rMotorEN(rMotorENPin);

Timeout     motorTimeout;
Timeout     lMotorTimeout;
Timeout     rMotorTimeout;
Timeout     lMotorISREnable;
Timeout     rMotorISREnable;

InterruptIn lBumpTop(lBumpTopPin);
InterruptIn lBumpBottom(lBumpBottomPin);
InterruptIn rBumpTop(rBumpTopPin);
InterruptIn rBumpBottom(rBumpBottomPin);

DigitalOut  bumperEnable(PC_6);

volatile bool lMotorBumped;
volatile bool rMotorBumped;


void lMotorBumperEnable(){
    lBumpTop.rise(&lMotorStop);
    lBumpBottom.rise(&lMotorStop);
    lBumpTop.fall(&lMotorReleased);
    lBumpBottom.fall(&lMotorReleased);
}

void rMotorBumperEnable(){
    rBumpTop.rise(&rMotorStop);
    rBumpBottom.rise(&rMotorStop);
    rBumpTop.fall(&rMotorReleased);
    rBumpBottom.fall(&rMotorReleased);
}

void motorStop(){
    lMotorEN.write(0.0f);
    rMotorEN.write(0.0f);
}

void lMotorStop(){
    lBumpTop.rise(NULL);
    lBumpBottom.rise(NULL);
    lBumpTop.fall(NULL);
    lBumpBottom.fall(NULL);
    lMotorEN.write(0.0f);
    lMotorBumped = true;
    lMotorISREnable.attach(&lMotorBumperEnable, 0.10f);
}

void rMotorStop(){
    rBumpTop.rise(NULL);
    rBumpBottom.rise(NULL);
    rBumpTop.fall(NULL);
    rBumpBottom.fall(NULL);
    rMotorEN.write(0.0f);
    rMotorBumped = true;
    rMotorISREnable.attach(&rMotorBumperEnable, 0.10f);
}

void lMotorUp(float speed){
    lMotorIN1 = 1;
    lMotorIN2 = 0;
    lMotorEN.write(speed);
}

void lMotorDown(float speed){
    lMotorIN1 = 0;
    lMotorIN2 = 1;
    lMotorEN.write(speed);
}

void rMotorUp(float speed){
    rMotorIN3 = 0;
    rMotorIN4 = 1;
    rMotorEN.write(speed);
}

void rMotorDown(float speed){
    rMotorIN3 = 1;
    rMotorIN4 = 0;
    rMotorEN.write(speed);
}

void lMotorReleased(){
    lMotorBumped = false;
    lBumpTop.rise(NULL);
    lBumpBottom.rise(NULL);
    lBumpTop.fall(NULL);
    lBumpBottom.fall(NULL);
    lMotorISREnable.attach(&lMotorBumperEnable, 0.10f);
}

void rMotorReleased(){
    rMotorBumped = false;
    rBumpTop.rise(NULL);
    rBumpBottom.rise(NULL);
    rBumpTop.fall(NULL);
    rBumpBottom.fall(NULL);
    rMotorISREnable.attach(&rMotorBumperEnable, 0.10f);
}