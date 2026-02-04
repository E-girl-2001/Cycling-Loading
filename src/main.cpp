
#include <Arduino.h>
#include <TaskScheduler.h>
#include "BasicStepperDriver.h"
#include "motor.h"

// ----------------- CONFIG -----------------
#define CURRENT_SAMPLE_TASK_PERIOD   100   // ms
#define SERIAL_TASK_PERIOD           100   // ms
#define MOTOR_TASK_PERIOD          1000   // ms
#define STEPPER_TASK_PERIOD         50   // ms
// the state machine drives the number of samples that the load cell takes 
// ~ force samples per rotation = Crank Period/LOAD_CELL_TASK_PERIOD
// (e.g 500ms/5ms = 100 samples per rotation)
#define LOAD_CELL_TASK_PERIOD        1   // ms
#define STATEMACHINE_TASK_PERIOD      5   // ms (fast tick; keep logic short)

// This task is only for testing without hardware
// COMMENT OUT WHEN LIMIT SWITCH IS INSTALLED AND MOTOR TRIGGERING SWITCH!!!!!!
#define FAKE_LIMIT_SWITCH_TASK_PERIOD 500 // ms (for testing without hardware)


// ----------------- SERIAL -----------------
#define PC_BAUD_RATE 115200


// ----------------- CURRENT MONITORING -----------------
#define CURRENT_THRESHOLD 2.0f // overcurrent threshold in Amps



// ----------------- STEPPER MOTOR -----------------
#define MOTOR_STEPS 200
#define RPM         30
#define MICROSTEPS  16 
#define DIR_PIN   53
#define STEP_PIN  52
static BasicStepperDriver stepper(MOTOR_STEPS, DIR_PIN, STEP_PIN);


// ----------------- MEASUREMENTS -----------------
float currentMeasure = 0.0;
int maxForce = 0;
int targetForce = 500;
int allowableForceError = 50;
int forceBuffer[1000];
size_t forceBufferIndex = 0;
int forceError = 0;
int nonZeroCount = 0;

// ------------------ LOAD CELL -----------------
#define LOAD_CELL_PIN A1




// ----------------- INIT HELPERS -----------------
static inline void initSerial() {
  Serial.begin(PC_BAUD_RATE);
}
// Load Cell implementation w/ Temp Pot
void initLoadCell() {
  pinMode(LOAD_CELL_PIN, INPUT);
}

void initStepper() {
  stepper.begin(RPM, MICROSTEPS);
}

// ----------------- LIMIT SWITCH / ISR (non-blocking, reliable) -----------------
#define LIMIT_SWITCH_PIN 2
static const uint32_t LIMIT_DEBOUNCE_US = 200UL * 1000UL; // 200 ms

volatile uint32_t cycleCount = 0;
volatile uint32_t lastLimitIsrUs = 0;
volatile bool limitEvent = false;

void limitSwitchCounterInterupt() {
  const uint32_t now = micros();  // safer than millis() inside ISR on many cores
  if ((uint32_t)(now - lastLimitIsrUs) >= LIMIT_DEBOUNCE_US) {
    lastLimitIsrUs = now;     // update first (minimizes chance of double count)
    cycleCount++;
    limitEvent = true;
  }
}

static inline void initLimitSwitch() {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN),
                  limitSwitchCounterInterupt,
                  LOW); // assuming switch connects to GND when triggered
}



// ----------------- SYSTEM STATE -----------------
enum class SystemState : uint8_t {
  Idle = 0,
  Running,
  Sending,
  Fault
};
static SystemState systemState = SystemState::Idle;



void printSystemState(SystemState state);


// ----------------- TASKS (callbacks) -----------------
void taskStateMachine();     // state machine tick task
void taskSerialRecieve();
void taskSerialTransmit();
void taskStepper();
void taskMotor();
void taskCurrentSample();
void taskFakeLimitSwitch(); // for testing without hardware
void taskLoadCell();
// ----------------- SCHEDULER + TASK OBJECTS -----------------
Scheduler runner;
// Create tasks: interval, iterations, callback
Task tStateMachine (STATEMACHINE_TASK_PERIOD, TASK_FOREVER, &taskStateMachine);
Task tSerialRx     (SERIAL_TASK_PERIOD,       TASK_FOREVER, &taskSerialRecieve);
Task tSerialTx     (SERIAL_TASK_PERIOD,       TASK_FOREVER, &taskSerialTransmit);
Task tCurrent      (CURRENT_SAMPLE_TASK_PERIOD, TASK_FOREVER, &taskCurrentSample);
Task tStepper      (STEPPER_TASK_PERIOD,      TASK_FOREVER, &taskStepper);
Task tMotor        (MOTOR_TASK_PERIOD,        TASK_FOREVER, &taskMotor);
Task tLoadCell     (LOAD_CELL_TASK_PERIOD,     TASK_FOREVER, &taskLoadCell);





// ----------------- FAKE LIMIT SWITCH TASK (for testing without hardware) -----------------
// Immitate the counter switch being triggered for testing without hardware to click the switch
Task tFakeLimitSwitch (FAKE_LIMIT_SWITCH_TASK_PERIOD, TASK_FOREVER, &taskFakeLimitSwitch);

void taskFakeLimitSwitch() {
  // Simulate a limit switch event for testing without hardware
  limitEvent = true;
}



// ----------------- TASK IMPLEMENTATIONS -----------------
// STATE MACHINE
void taskStateMachine() {
  
  switch (systemState) {
    case SystemState::Idle:
    // TODO: add idle behavior if needed
    // Disable actuators, keep comms alive
    setMotorSpeed(0);
      break;

    case SystemState::Running: {
      // TODO: add force values to a buffer when the limit switch event occurs
      // Store force measurement in buffer

      if (limitEvent) {
        // Limit switch was triggered; reset event flag
        limitEvent = false;
        systemState = SystemState::Sending;
      } else {
        // printSystemState(systemState);
      }

    } break;

    case SystemState::Sending: {
      // Send the maximum value collected in the force buffer over serial to the PC and then clear the buffer
        maxForce = 0;
        for (size_t i = 0; i < 1000; i++) {
          if (forceBuffer[i] != 0) {
            nonZeroCount++;
          }

          if (forceBuffer[i] > maxForce) {
            maxForce = forceBuffer[i];
          }
          forceBuffer[i] = 0; // clear buffer
        }

        forceError = targetForce - maxForce;
        printSystemState(systemState);
        nonZeroCount = 0;
        forceBufferIndex = 0; // reset index
        systemState = SystemState::Running; // return to running after sending
      }

      break;


    case SystemState::Fault:
      // safe stop; keep comms and sensing alive
      setMotorSpeed(0);
      break;
  }
}

void printSystemState(SystemState state) {
  Serial.print(maxForce);
  Serial.print(", ");
  Serial.print(targetForce);
  Serial.print(", ");
  Serial.print(forceError);
  Serial.print(", ");
  Serial.print(nonZeroCount);
  Serial.print(", ");
  Serial.print(currentMeasure, 2);
  Serial.print(", ");

  switch (state) {
    case SystemState::Idle:
      Serial.print("Idle");
      break;
    case SystemState::Running:
      Serial.print("Running");
      break;
    case SystemState::Sending:
      Serial.print("Sending");
      break;
    case SystemState::Fault:
      Serial.print("Fault");
      break;
    default:
      Serial.print("Unknown");
      break;
  }
  Serial.println();
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
  // Sample current and check for overcurrent condition on the Crank Motor
  currentMeasure = readCurrent();
  if (currentMeasure > CURRENT_THRESHOLD) {
    systemState = SystemState::Fault; // overcurrent fault
  }
}


void taskStepper() {

  // Bang bang control to adjust stepper position based on force error
  if (systemState == SystemState::Idle || systemState == SystemState::Fault) {
    stepper.stop();
    return;
  }
  if (maxForce < (targetForce - allowableForceError)) {
    stepper.move(10);
  } else if (maxForce > (targetForce + allowableForceError)) {
    // step backward if above target force
    stepper.move(-10);
  } else {
    // within target range; do not step
    stepper.stop();
  }

  // Proportional control to adjust stepper position based on force error
  // float kp = 0.1; // Proportional gain
  // int stepAdjustment = kp * forceError;
  // if (stepAdjustment < 1 && stepAdjustment > -1) {
  //   stepAdjustment = 0; // Limit maximum step adjustment
  // }
  // stepper.move(stepAdjustment);

}


void taskLoadCell() {
  // Read force value from load cell (via temp pot for this prototype)
  forceBuffer[forceBufferIndex] = analogRead(LOAD_CELL_PIN);
  forceBufferIndex = (forceBufferIndex + 1) % 1000;

}


void taskMotor() {
  // turn the motor on or off based on system state
  if (systemState == SystemState::Running || systemState == SystemState::Sending) {
    setMotorSpeed(255);
  } else {
    setMotorSpeed(0);
  }
}


void taskSerialTransmit() {
  // Transmit current system status periodically
  printSystemState(systemState);
}




// ----------------- ARDUINO SETUP/LOOP -----------------
void setup() {
  // Initialize subsystems
  initCurrentMonitoring();
  initMotor();
  initLimitSwitch();
  initSerial();
  initLoadCell();
  initStepper();

  // Add tasks to scheduler chain (execution order = add order)
  runner.addTask(tStateMachine);
  runner.addTask(tSerialRx);
  runner.addTask(tCurrent);
  runner.addTask(tStepper);
  runner.addTask(tMotor);
  runner.addTask(tLoadCell);

  // Enable the relevant tasks
  tStateMachine.enable();
  tSerialRx.enable();
  tCurrent.enable();
  tStepper.enable();
  tMotor.enable();
  tLoadCell.enable();


  // Temporary: for testing without hardware limit switch
  // remove when the limit switch is attached and is triggered periodically by the crank
  runner.addTask(tFakeLimitSwitch);
  // tFakeLimitSwitch.enable();

  // Start in running state for testing
  // Should be set to Idle and started via serial command in real use
  systemState = SystemState::Idle;

}

void loop() {
  runner.execute();
}


