#include "mbed.h"

extern Serial PC_serial;
//redefine serial printf to simple "PCSERIAL"
#define PCSERIAL(...) { PC_serial.printf(__VA_ARGS__); }