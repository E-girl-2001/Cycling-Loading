
#include <Arduino.h>
#include "BasicStepperDriver.h"

// --- Configure to match your hardware ---
#define MOTOR_STEPS 200
#define RPM         120
#define MICROSTEPS  2 

// STEP/DIR pins (your example used DIR=24 STEP=22)
#define DIR_PIN   24
#define STEP_PIN  22


// Instantiate driver
static BasicStepperDriver stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN);

void initStepper() {
  // Configure speed and microstepping
  stepper.begin(RPM, MICROSTEPS);

}

void setup () {
  initStepper();
}

void loop () {
  // Example: Move 1 revolution forward and backward
  stepper.rotate(360);
  delay(1000);
  stepper.rotate(-360);
  delay(1000);
}