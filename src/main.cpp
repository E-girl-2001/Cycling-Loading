
#include <Arduino.h>
#include <TaskScheduler.h>

#include "motor.h"
#include "stepper.h"

// ----------------- CONFIG -----------------
#define CURRENT_SAMPLE_TASK_PERIOD   10   // ms
#define SERIAL_TASK_PERIOD           100   // ms
#define MOTOR_TASK_PERIOD          1000   // ms
#define STEPPER_TASK_PERIOD         50   // ms
#define STATEMACHINE_TASK_PERIOD      5   // ms (fast tick; keep logic short)
#define LOAD_CELL_TASK_PERIOD         5

#define PC_BAUD_RATE 115200

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
  Calibrating,
  Running,
  Fault
};

static SystemState systemState = SystemState::Idle;

// ----------------- MEASUREMENTS -----------------
float currentMeasure = 0.0;
int forceMeasure   = 0;   // keep as placeholder unless you update elsewhere
int max_force = 200;
int min_force = 150;

// ----------------- INIT HELPERS -----------------
static inline void initSerial() {
  Serial.begin(PC_BAUD_RATE);
}

static inline void initLimitSwitch() {
  pinMode(limitSwitchPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(limitSwitchPin),
                  limitSwitchCounterInterupt,
                  FALLING);
}

// ----------------- TASKS (callbacks) -----------------
void taskStateMachine();     // state machine tick task
void taskSerialSend();
void taskSerialRecieve();
void taskStepper();
void taskMotor();
void taskCurrentSample();
void taskLoadCell();

// ----------------- SCHEDULER + TASK OBJECTS -----------------
Scheduler runner;

// Create tasks: interval, iterations, callback
Task tStateMachine (STATEMACHINE_TASK_PERIOD, TASK_FOREVER, &taskStateMachine);
Task tSerialRx     (SERIAL_TASK_PERIOD,       TASK_FOREVER, &taskSerialRecieve);
Task tSerialTx     (SERIAL_TASK_PERIOD,       TASK_FOREVER, &taskSerialSend);
Task tCurrent      (CURRENT_SAMPLE_TASK_PERIOD, TASK_FOREVER, &taskCurrentSample);
Task tStepper      (STEPPER_TASK_PERIOD,      TASK_FOREVER, &taskStepper);
Task tMotor        (MOTOR_TASK_PERIOD,        TASK_FOREVER, &taskMotor);
Task tLoadCell     (LOAD_CELL_TASK_PERIOD,        TASK_FOREVER, &taskLoadCell);







// ----------------- ARDUINO SETUP/LOOP -----------------
void setup() {
  // Your existing module inits
  initCurrentMonitoring();
  initMotor();
  initStepper();
  initLimitSwitch();
  initSerial();

  // Add tasks to scheduler chain (execution order = add order)
  runner.addTask(tStateMachine);
  runner.addTask(tSerialRx);
  runner.addTask(tSerialTx);
  runner.addTask(tCurrent);
  runner.addTask(tStepper);
  runner.addTask(tMotor);
  runner.addTask(tLoadCell);


  tStateMachine.enable();
  systemState = SystemState::Calibrating;



}

void loop() {
  runner.execute();
}



// ----------------- TASK IMPLEMENTATIONS -----------------
void taskStateMachine() {
  
  switch (systemState) {
    case SystemState::Idle:
      // Safe stop while idle
      tSerialRx.enableIfNot();
      tSerialTx.enableIfNot();
      tLoadCell.enableIfNot();
      tCurrent.enableIfNot();

      tStepper.disable();
      tMotor.disable();

      break;

    case SystemState::Calibrating: {
      
      tSerialRx.enableIfNot();
      tSerialTx.enableIfNot();
      tLoadCell.enableIfNot();
      tCurrent.enableIfNot();
      // TODO: stall program to wait for a spesfic force measure input???
      setMotorSpeed(255);
      // Control moving the steppper forward until the correct force is measured. 

      if (forceMeasure > max_force) {
        stepBackward();

      } else if (forceMeasure < min_force) {
        Serial.println("stepping");
        stepForward();
      } else {
        systemState = SystemState::Idle;
      }
    } break;

    case SystemState::Running:
      // normal operation; motor/stepper tasks will run
      // add fault checks here if needed
      break;


    case SystemState::Fault:
      // safe stop; keep comms and sensing alive
      break;
  }
}


void taskSerialRecieve() {
  // Minimal command parser.
  // 'C' from Idle -> Calibrating
  // 'R' -> Running
  // 'B' -> stop/back to Idle
  // 'F' -> Fault
  while (Serial.available() > 0) {
    const char cmd = (char)Serial.read();
    if (cmd == 'C' && systemState == SystemState::Idle) {
      systemState = SystemState::Calibrating;
    }
    else if (cmd == 'R') {
      systemState = SystemState::Running;
    }
    else if (cmd == 'B') {
      systemState = SystemState::Idle;
    }
    else if (cmd == 'F') {
      systemState = SystemState::Fault;
    }
  }
}




void taskSerialSend() {
  Serial.print(F("CycleCount, "));
  Serial.print(cycleCount);
  Serial.print(F(", Current, "));
  Serial.print(currentMeasure);
  Serial.print(F(", Force, "));
  Serial.print(forceMeasure);
  Serial.print(", State, ");
  switch (systemState) {
    case SystemState::Idle:
      Serial.print("Idle");
      break;

    case SystemState::Calibrating:
      Serial.print("Calibrating");
      break;

    case SystemState::Running:
      Serial.print("Running");
      break;


    case SystemState::Fault:
      Serial.print("Fault");
      break;
  }
  Serial.println("");
}



void taskCurrentSample() {
  currentMeasure = readCurrent();
}

void taskStepper() {
  // TODO
}

void taskMotor() {
  // Minimal example: enforce speed depending on state
  setMotorSpeed(255);
}


// Load Cell implementation w/ Temp Pot
void initLoadCell() {
  pinMode(A1, INPUT);
}

void taskLoadCell() {
  forceMeasure = analogRead(A1);
}