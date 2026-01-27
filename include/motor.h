// Current Monitoring Header File

#include <stdint.h>
#include <stdlib.h>
#include <Arduino.h>

void initCurrentMonitoring();
void initMotor();
void setMotorSpeed(uint8_t);
float readCurrent();

