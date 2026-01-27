
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "stepper.h"

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

void stepForward() {
  // Move one “microstep unit” as the library counts steps using MOTOR_STEPS*MICROSTEPS in examples [1](https://github.com/laurb9/StepperDriver/blob/master/examples/BasicStepperDriver/BasicStepperDriver.ino)
  stepper.move(10);
}

void stepBackward() {
  stepper.move(-10);
}