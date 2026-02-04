// Current Monitoring Definintions

#include "motor.h"

#define IN1 11
#define IN2 12
#define ENA 10
#define CURRENT_SENSE_PIN A0
#define SENSE_RESISTOR_VALUE 1.65f



void initCurrentMonitoring() {
    pinMode(CURRENT_SENSE_PIN, INPUT);
}

void initMotor() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

}

void setMotorSpeed(uint8_t speed) {
  /// 0-255 for the PWM signal
  analogWrite(ENA, speed);
  // Direction setting (Not relevent for this application as translated into linear motion)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}


float readCurrent() {
  int adc = analogRead(CURRENT_SENSE_PIN);
  float voltage = (adc / 1023.0f) * 5.0f;
  return voltage / SENSE_RESISTOR_VALUE;
}