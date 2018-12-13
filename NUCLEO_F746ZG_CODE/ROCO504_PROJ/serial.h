#include "mbed.h"

extern Serial PC_serial;
#define PCSERIAL(...) { PC_serial.printf(__VA_ARGS__); }