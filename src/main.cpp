
#include <Arduino.h>
#include <TaskScheduler.h>
#include "BasicStepperDriver.h"
#include "motor.h"

// ----------------- CONFIG -----------------
#define CURRENT_SAMPLE_TASK_PERIOD   100   // ms
#define SERIAL_TASK_PERIOD           100   // ms
#define MOTOR_TASK_PERIOD          1000   // ms
#define STEPPER_TASK_PERIOD         50   // ms
#define STATEMACHINE_TASK_PERIOD      5   // ms (fast tick; keep logic short)

#define PC_BAUD_RATE 115200

// Stepper Motor Definitions
#define MOTOR_STEPS 200
#define RPM         100
#define MICROSTEPS  2 
#define DIR_PIN   24
#define STEP_PIN  22
static BasicStepperDriver stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN);

#define LIMIT_SWITCH_PIN 2

// ----------------- MEASUREMENTS -----------------
float currentMeasure = 0.0;
int maxForce = 0;
int targetForce = 500;
int allowableForceError = 50;
int forceBuffer[1000];
size_t forceBufferIndex = 0;




// ----------------- INIT HELPERS -----------------
static inline void initSerial() {
  Serial.begin(PC_BAUD_RATE);
}
// Load Cell implementation w/ Temp Pot
void initLoadCell() {
  pinMode(A1, INPUT);
}

void limitSwitchCounterInterupt();


static inline void initLimitSwitch() {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN),
                  limitSwitchCounterInterupt,
                  FALLING);
}

// ----------------- LIMIT SWITCH / ISR -----------------
const byte limitSwitchPin = 2;
static const uint32_t LIMIT_DEBOUNCE_MS = 50;

volatile uint32_t cycleCount = 0;
volatile uint32_t lastLimitIsrMs = 0;
volatile bool limitEvent = false;

void limitSwitchCounterInterupt() {
  uint32_t now = millis(); // short gating; keep ISR minimal
  if ((uint32_t)(now - lastLimitIsrMs) >= LIMIT_DEBOUNCE_MS) {
    cycleCount++;
    limitEvent = true;
    lastLimitIsrMs = now;
  }
}




// ----------------- SYSTEM STATE -----------------
enum class SystemState : uint8_t {
  Idle = 0,
  Running,
  Sending,
  Fault
};
static SystemState systemState = SystemState::Idle;


// ----------------- TASKS (callbacks) -----------------
void taskStateMachine();     // state machine tick task
void taskSerialRecieve();
void taskStepper();
void taskMotor();
void taskCurrentSample();

// ----------------- SCHEDULER + TASK OBJECTS -----------------
Scheduler runner;
// Create tasks: interval, iterations, callback
Task tStateMachine (STATEMACHINE_TASK_PERIOD, TASK_FOREVER, &taskStateMachine);
Task tSerialRx     (SERIAL_TASK_PERIOD,       TASK_FOREVER, &taskSerialRecieve);
Task tCurrent      (CURRENT_SAMPLE_TASK_PERIOD, TASK_FOREVER, &taskCurrentSample);
Task tStepper      (STEPPER_TASK_PERIOD,      TASK_FOREVER, &taskStepper);
Task tMotor        (MOTOR_TASK_PERIOD,        TASK_FOREVER, &taskMotor);







// ----------------- TASK IMPLEMENTATIONS -----------------

// STATE MACHINE
void taskStateMachine() {
  
  switch (systemState) {
    case SystemState::Idle:
    // TODO: add idle behavior if needed
      break;

    case SystemState::Running: {
      // TODO: add force values to a buffer when the limit switch event occurs
      // Store force measurement in buffer
      forceBuffer[forceBufferIndex] = analogRead(A1);
      forceBufferIndex = (forceBufferIndex + 1) % 1000;
      
      
      if (limitEvent) {
        // Limit switch was triggered; reset event flag
        limitEvent = false;
        systemState = SystemState::Sending;
      }

    } break;

    case SystemState::Sending: {
      // Send the maximum value collected in the force buffer over serial to the PC and then clear the buffer
        maxForce = 0;
        for (size_t i = 0; i < 1000; i++) {
          if (forceBuffer[i] > maxForce) {
            maxForce = forceBuffer[i];
          }
          forceBuffer[i] = 0; // clear buffer
        }

        Serial.print(F("Max Force: "));
        Serial.println(maxForce);
        forceBufferIndex = 0; // reset index
        systemState = SystemState::Running; // return to running after sending
      }

      break;


    case SystemState::Fault:
      // safe stop; keep comms and sensing alive
      break;
  }
}



void taskSerialRecieve() {
  // Minimal command parser.
  // 'R' -> Running
  // 'I' -> stop/back to Idle
  // 'F' -> Fault
  while (Serial.available() > 0) {
    const char cmd = (char)Serial.read();
    if (cmd == 'R') {
      systemState = SystemState::Running;
    }
    else if (cmd == 'I') {
      systemState = SystemState::Idle;
    }
    else if (cmd == 'F') {
      systemState = SystemState::Fault;
    }
  }
}


void taskCurrentSample() {
  currentMeasure = readCurrent();
  if (currentMeasure > 3.0f) {
    systemState = SystemState::Fault; // overcurrent fault
  }
}


void taskStepper() {

  if (maxForce < (targetForce - allowableForceError)) {
    stepper.move(10);
  } else if (maxForce > (targetForce + allowableForceError)) {
    // step backward if above target force
    stepper.move(-10);
  } else {
    // within target range; do not step
    stepper.stop();
  }

}



void taskMotor() {
  if (systemState == SystemState::Running || systemState == SystemState::Sending) {
    setMotorSpeed(255);
  } else {
    setMotorSpeed(0);
  }
}







// ----------------- ARDUINO SETUP/LOOP -----------------
void setup() {
  // Your existing module inits
  initCurrentMonitoring();
  initMotor();
  initLimitSwitch();
  initSerial();

  // Add tasks to scheduler chain (execution order = add order)
  runner.addTask(tStateMachine);
  runner.addTask(tSerialRx);
  runner.addTask(tCurrent);
  runner.addTask(tStepper);
  runner.addTask(tMotor);

  stepper.begin(RPM, MICROSTEPS);

  tStateMachine.enable();
  tSerialRx.enable();
  tCurrent.enable();
  tStepper.enable();
  tMotor.enable();



  systemState = SystemState::Running;

}

void loop() {
  runner.execute();
}


