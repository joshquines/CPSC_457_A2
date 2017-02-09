#include "devices/Screen.h"
#include "devices/Serial.h"

bool SerialDevice::gdb;           // Serial.h
bool DebugDevice::valid = false;  // Serial.h

char Screen::buffer[xmax * ymax]; // Screen.h
char* Screen::video = nullptr;    // Screen.h 
