#include "mbed.h"

/*==============
DC MOTOR CONTROL
==============*/
extern DigitalOut  lMotorIN1;
extern DigitalOut  lMotorIN2;
extern PwmOut      lMotorEN;

extern DigitalOut  rMotorIN3;
extern DigitalOut  rMotorIN4;
extern PwmOut      rMotorEN;

extern Timeout     motorTimeout;
extern Timeout     lMotorTimeout;
extern Timeout     rMotorTimeout;
extern Timeout     lMotorISREnable;
extern Timeout     rMotorISREnable;

extern InterruptIn lBumpTop;
extern InterruptIn lBumpBottom;
extern InterruptIn rBumpTop;
extern InterruptIn rBumpBottom;

extern DigitalOut  bumperEnable;

extern bool lMotorBumped;
extern bool rMotorBumped;


extern void lMotorBumperEnable();
extern void rMotorBumperEnable();
extern void motorStop();
extern void lMotorStop();
extern void rMotorStop();
extern void lMotorUp(float speed);
extern void lMotorDown(float speed);
extern void rMotorUp(float speed);
extern void rMotorDown(float speed);
extern void lMotorReleased();
extern void rMotorReleased();

