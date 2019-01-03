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

/*======
TIMEOUTS
======*/
Timeout     motorTimeout;
Timeout     lMotorTimeout;
Timeout     rMotorTimeout;
Timeout     lMotorISREnable;
Timeout     rMotorISREnable;

/*========
INTERRUPTS
========*/
InterruptIn lBumpTop(lBumpTopPin);
InterruptIn lBumpBottom(lBumpBottomPin);
InterruptIn rBumpTop(rBumpTopPin);
InterruptIn rBumpBottom(rBumpBottomPin);

/*==============
POWER TO BUMPERS
==============*/
DigitalOut  bumperEnable(PC_6);

volatile bool lMotorBumped;
volatile bool rMotorBumped;

/*========================
MOTOR AND BUMPER FUNCTIONS
========================*/
//function to enable left bumpers
void lMotorBumperEnable(){
    lBumpTop.rise(&lMotorStop);
    lBumpBottom.rise(&lMotorStop);
    lBumpTop.fall(&lMotorReleased);
    lBumpBottom.fall(&lMotorReleased);
}

//function to enable right bumpers
void rMotorBumperEnable(){
    rBumpTop.rise(&rMotorStop);
    rBumpBottom.rise(&rMotorStop);
    rBumpTop.fall(&rMotorReleased);
    rBumpBottom.fall(&rMotorReleased);
}

//function to stop both motors
void motorStop(){
    lMotorEN.write(0.0f);
    rMotorEN.write(0.0f);
}

//function to stop left motor and try and get rid of switch debounce by..
//..disabling interrupts and then attach interrupts again after 100ms
void lMotorStop(){
    lBumpTop.rise(NULL);
    lBumpBottom.rise(NULL);
    lBumpTop.fall(NULL);
    lBumpBottom.fall(NULL);
    lMotorEN.write(0.0f);
    lMotorBumped = true;
    lMotorISREnable.attach(&lMotorBumperEnable, 0.10f);
}

//function to stop right motor and try and get rid of switch debounce by..
//..disabling interrupts and then attach interrupts again after 100ms
void rMotorStop(){
    rBumpTop.rise(NULL);
    rBumpBottom.rise(NULL);
    rBumpTop.fall(NULL);
    rBumpBottom.fall(NULL);
    rMotorEN.write(0.0f);
    rMotorBumped = true;
    rMotorISREnable.attach(&rMotorBumperEnable, 0.10f);
}

//function to move left motor up at set speed (0-1) where 1 is full power
void lMotorUp(float speed){
    lMotorIN1 = 1;
    lMotorIN2 = 0;
    lMotorEN.write(speed);
}

//function to move left motor down at set speed (0-1) where 1 is full power
void lMotorDown(float speed){
    lMotorIN1 = 0;
    lMotorIN2 = 1;
    lMotorEN.write(speed);
}

//function to move right motor up at set speed (0-1) where 1 is full power
void rMotorUp(float speed){
    rMotorIN3 = 0;
    rMotorIN4 = 1;
    rMotorEN.write(speed);
}

//function to move right motor down at set speed (0-1) where 1 is full power
void rMotorDown(float speed){
    rMotorIN3 = 1;
    rMotorIN4 = 0;
    rMotorEN.write(speed);
}

//function for when left motor releases off a bumper
void lMotorReleased(){
    lMotorBumped = false;
    lBumpTop.rise(NULL);
    lBumpBottom.rise(NULL);
    lBumpTop.fall(NULL);
    lBumpBottom.fall(NULL);
    lMotorISREnable.attach(&lMotorBumperEnable, 0.10f);
}

//function for when right motor releases off a bumper
void rMotorReleased(){
    rMotorBumped = false;
    rBumpTop.rise(NULL);
    rBumpBottom.rise(NULL);
    rBumpTop.fall(NULL);
    rBumpBottom.fall(NULL);
    rMotorISREnable.attach(&rMotorBumperEnable, 0.10f);
}