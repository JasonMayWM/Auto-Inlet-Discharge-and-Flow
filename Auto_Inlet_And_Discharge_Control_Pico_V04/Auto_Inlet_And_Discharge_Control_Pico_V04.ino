// Raspberry Pi Pico and TB6600 stepper driver set to 1A.
// Nema17 stepper with 51:1 gearbox, driven with 1/8 microstepPinInletg.

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include "hardware/adc.h"

// === Global State Variables ===
unsigned long ticktime = 0;
int top = -1;  // Stack is initially empty
bool reset = false;

// === ADC & Sensor Constants ===
const float MY_ADC_RESOLUTION = 4095.0;

// Inlet Sensor
const float INLET_SENSOR_SCALE = 0.99;
const float INLET_SENSOR_RANGE = 2.0;  // -1 to +1 bar
const float INLET_SENSOR_OFFSET = -1.0;
const float INLET_PRESSURE_CONV = (INLET_SENSOR_RANGE * INLET_SENSOR_SCALE / MY_ADC_RESOLUTION);

// Discharge Sensor
const float DISCHARGE_SENSOR_SCALE = 1.0;
const float DISCHARGE_SENSOR_RANGE = 10.0;  // 0 to 10 bar
const float DISCHARGE_SENSOR_OFFSET = -0.08;
const float DISCHARGE_PRESSURE_CONV = (DISCHARGE_SENSOR_RANGE * DISCHARGE_SENSOR_SCALE / MY_ADC_RESOLUTION);

// === Pin Definitions ===
// ADC Pins
const int inletAnalogPin = 0;      // GPIO26
const int dischargeAnalogPin = 1;  // GPIO27

// Inlet Motor Control Pins
const int stepPinInlet = 4;
const int dirPinInlet = 3;
const int enaPinInlet = 2;

// Discharge Motor Control Pins
const int stepPinDischarge = 8;
const int dirPinDischarge = 7;
const int enaPinDischarge = 6;

// === Motor Control Variables ===
// Step counters
long stepCountInlet = 0;
long stepCountDischarge = 0;
long stepCountAccInlet = 0;
long stepCountAccDischarge = 0;

// Maximum steps
long maxStepsInlet = 325000;
long maxStepsDischarge = 350000;

// Timing
unsigned long motorLastStepTime = 0;
unsigned long motorLastStepTimeDischarge = 0;

// Step intervals
long intervalInlet = 200;
long intervalRunInlet = 200;
long initialIntervalInlet = 1000;
long intervalDischarge = 200;     //This is the base step interval that results from the acceleration profile. It represents the current motor speed after accelerating, but before any adjustments for slowing down near the target pressure.
long intervalRunDischarge = 200;  //This is the final step interval actually used to control the motor. It’s based on intervalRunDischarge, but modified when you're close to the target pressure.
long initialIntervalDischarge = 1000;

// Acceleration
long accelerationStepsInlet = 2000;
long accelerationStepsDischarge = 1500;

// Motor Direction Control
bool motorDirectionInlet = true;  // Set default direction. true = Clockwise, false = counterclockwise
bool previousDirectionInlet = true;
bool motorDirectionDischarge = true;  // Set default direction. true = Clockwise, false = counterclockwise
bool previousDirectionDischarge = true;

// === Pressure Variables ===
volatile float currentPressureInlet = 0.0;
volatile float currentPressureDischarge = 0.0;

float setPressureInlet = 0.0;
float setPressureDischarge = 0.0;
float avg = 0.0;
float avgVoltage = 0.0;
float pressureToleranceInlet = 0.02;
float pressureToleranceDischarge = 0.05;
float maximumPressureInlet = 0.0;
float minimumPressureDischarge = 0.2;

bool targetPressure = false;
bool targetDischargePressure = false;
bool notifyInlet = false;
bool notifyDischarge = false;

bool controlEnableInlet = true;
bool controlEnableDischarge = true;
int avgSampTime = 10000;  // Sampling time in microseconds

// === Serial Communication ===
const byte numChars = 16;
char receivedChars[numChars];
boolean newData = false;
float dataNumber = 0;
String serialStr;

// === Circular Buffer ===
const int arraySize = 200;

bool streamPressure = false;
unsigned long lastStreamTime = 0;
const unsigned long streamInterval = 1000;  // in milliseconds

struct CircularBuffer {
  int buffer[arraySize];
  int start = 0;
  int count = 0;

  void push(int value) {
    int idx = (start + count) % arraySize;
    buffer[idx] = value;
    if (count < arraySize) {
      count++;
    } else {
      start = (start + 1) % arraySize;
    }
  }

  float average() {
    if (count == 0) return 0;
    float sum = 0;
    for (int i = 0; i < count; i++) {
      sum += buffer[(start + i) % arraySize];
    }
    return sum / count;
  }
};

CircularBuffer dischargeFilo;  // Circular array for discharge pressure sensor
CircularBuffer inletFilo;      // Circular array for inlet pressure sensor

void pressureSensorCore1();  // Core 1 Function Declaration

void setup() {
  pinMode(stepPinInlet, OUTPUT);
  pinMode(dirPinInlet, OUTPUT);
  pinMode(enaPinInlet, OUTPUT);
  pinMode(stepPinDischarge, OUTPUT);
  pinMode(dirPinDischarge, OUTPUT);
  pinMode(enaPinDischarge, OUTPUT);
  Serial.begin(115200);
digitalWrite(STEP_INLET_PIN, LOW);
digitalWrite(STEP_DISCHARGE_PIN, LOW);

  adc_init();         // Initialize ADC pins
  adc_gpio_init(26);  // A0 (GPIO26)
  adc_gpio_init(27);  // A1 (GPIO27)
  multicore_launch_core1(pressureSensorCore1);

  while (!Serial)
    ;
  Serial.println("Inlet and Discharge Pressure Control Software Version 0.01");
  Serial.println("Enter 'help' to list available serial commands");
}

void loop() {
  serial();
  pressureControlInlet();
  pressureControlDischarge();
    if (streamPressure && millis() - lastStreamTime >= streamInterval) {
    lastStreamTime = millis();
    printBuffer(inletFilo, "Inlet");
    printBuffer(dischargeFilo, "Discharge");
  }
}

void pressureSensorCore1() {
  while (true) {
    pressureSensor(inletFilo, inletAnalogPin, currentPressureInlet, true);
    pressureSensor(dischargeFilo, dischargeAnalogPin, currentPressureDischarge, false);
  }
}

void pressureControlInlet() {
  digitalWrite(enaPinInlet, targetPressure ? HIGH : LOW);
  // Check if zero or negative target pressure is achieved
  if (!reset && setPressureInlet < 0 && fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet) {
    targetPressure = true;
    if (!notifyInlet) {
      Serial.printf("Inlet target pressure %.2f bar achieved\n",currentPressureInlet);
      notifyInlet = true;
    }
  }
  // Check if zero position is achieved
  else if (setPressureInlet == 0 && fabs(currentPressureInlet - setPressureInlet) >= maximumPressureInlet && stepCountInlet == 0 && !reset) {
    targetPressure = true;
    if (!notifyInlet) {
      Serial.println("Inlet zero position achieved");
      notifyInlet = true;
    }
  }
  // If neither condition is met
  else {
    targetPressure = false;
  }
  // Adjust pressure if needed
  if ((int32_t)micros() - motorLastStepTime >= intervalInlet && !targetPressure && controlEnableInlet && !reset) {
    adjustPressureInlet();
  }
}

void pressureControlDischarge() {
  digitalWrite(enaPinDischarge, targetDischargePressure ? HIGH : LOW);
  // Check if the target discharge pressure is achieved
  if (!reset && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge) {
    targetDischargePressure = true;
    if (!notifyDischarge) {
      Serial.printf("Discharge target pressure %.2f bar achieved\n",currentPressureDischarge);
      notifyDischarge = true;
    }
  }
  // Check if the discharge zero position is achieved
  else if (setPressureDischarge == 0 && fabs(currentPressureDischarge - setPressureDischarge) <= minimumPressureDischarge && stepCountDischarge == 0 && !reset) {
    targetDischargePressure = true;
    if (!notifyDischarge) {
      Serial.println("Discharge zero position achieved");
      notifyDischarge = true;
    }
  }
  // Otherwise, target pressure is not achieved
  else {
    targetDischargePressure = false;
  }
  // Adjust pressure if needed
  if ((int32_t)micros() - motorLastStepTimeDischarge >= intervalDischarge && !targetDischargePressure && controlEnableDischarge && !reset) {
    adjustPressureDischarge();
  }
}


void adjustPressureInlet() {
  if (setPressureInlet != 0 && stepCountInlet > 0 && ((currentPressureInlet > setPressureInlet && setPressureInlet > 0) || (currentPressureInlet < setPressureInlet && setPressureInlet < 0))) {
    moveMotorInlet(false);  // Move motor backward if pressure exceeds target (positive or negative)
  } else if (setPressureInlet != 0 && stepCountInlet < maxStepsInlet && ((currentPressureInlet < setPressureInlet && setPressureInlet > 0) || (currentPressureInlet > setPressureInlet && setPressureInlet < 0))) {
    moveMotorInlet(true);  // Move motor forward to reach target pressure
  } else if (stepCountInlet >= maxStepsInlet && setPressureInlet != 0) {
    controlEnableInlet = false;  // Stop Motor
    Serial.println("Inlet motor at maximum step count. Motor disabled, reset pressure to '0' and then 'start' to enable.");
  } else if (setPressureInlet == 0 && stepCountInlet > 0) {
    moveMotorInlet(false);  // Return motor to zero position
  } else if (reset == true) {
    resetMotor();  // Reset motor
  }
}


void adjustPressureDischarge() {
  if (setPressureDischarge != 0 && stepCountDischarge > 0 && ((currentPressureDischarge > setPressureDischarge && setPressureDischarge > 0) || (currentPressureDischarge < setPressureDischarge && setPressureDischarge < 0))) {
    moveMotorDischarge(false);
  } else if (setPressureDischarge != 0 && stepCountDischarge < maxStepsDischarge && ((currentPressureDischarge < setPressureDischarge && setPressureDischarge > 0) || (currentPressureDischarge > setPressureDischarge && setPressureDischarge < 0))) {
    moveMotorDischarge(true);
  } else if (stepCountDischarge >= maxStepsDischarge && setPressureDischarge != 0) {
    controlEnableDischarge = false;  // Stop Motor
    Serial.println("Discharge motor at maximum step count. Motor disabled, reset pressure to '0' and then 'start' to enable.");
  } else if (setPressureDischarge == 0 && stepCountDischarge > 0) {
    moveMotorDischarge(false);
  } else if (reset == true) {
    resetMotorDischarge();
  }
}

// Function to set motor direction based on motorDirectionInlet value
void setMotorDirectionInlet(bool direction) {
  if (motorDirectionInlet) {
    // Invert direction if motorDirectionInlet is true
    digitalWrite(dirPinInlet, !direction);
  } else {
    // Otherwise, use the direction as is
    digitalWrite(dirPinInlet, direction);
  }

  // Check if the direction has changed
  if (direction != previousDirectionInlet) {
    stepCountAccInlet = 0;               // Reset stepCountAcc to zero on direction change
    previousDirectionInlet = direction;  // Update the previous direction
  }
}


void calculateIntervalInlet() {
  float errorInlet = fabs(currentPressureInlet - setPressureInlet);
  if (stepCountAccInlet < accelerationStepsInlet) {
    float progress = (float)stepCountAccInlet / accelerationStepsInlet;
    intervalInlet = initialIntervalInlet - (progress * (initialIntervalInlet - intervalRunInlet));
    stepCountAccInlet++;
  }

  // Check if within 0.1 bar of target pressure
  if (errorInlet < 0.2 && setPressureInlet != 0 || currentPressureInlet <= -0.25) {
    intervalInlet = intervalRunInlet * 16.0;  // Slow down (double the interval)
  } else if (errorInlet < 0.15 && setPressureInlet != 0) {
    intervalInlet = intervalRunInlet * 40.0;
  } else {
    intervalInlet = intervalRunInlet;
  }
}

// Updated moveMotor function with acceleration
void moveMotorInlet(bool direction) {
  if (controlEnableInlet) {
    setMotorDirectionInlet(direction);  // Use setmotorDirectionInlet to control dirPinInlet
    calculateIntervalInlet();           // Update interval based on step count for acceleration
    digitalWrite(stepPinInlet, HIGH);
    delayMicroseconds(2);
    digitalWrite(stepPinInlet, LOW);
    motorLastStepTime = (int32_t)micros();
    stepCountInlet += direction ? 1 : -1;
    if (notifyInlet == true) {
      Serial.println("adjusting...");
      notifyInlet = false;
    }
  }
}

void resetMotor() {
  if (currentPressureInlet > maximumPressureInlet) {
    moveMotorInlet(false);  // false equals the direction required to wind the pressure regulator to minimum
  } else {
    reset = false;
    stepCountInlet = 0;
    stepCountAccInlet = 0;
    stepCountAccDischarge = 0;
    setPressureInlet = 0;
    Serial.println("reset complete");
    // controlEnable = false;
  }
}

void setMotorDirectionDischarge(bool direction) {
  if (motorDirectionDischarge) {
    digitalWrite(dirPinDischarge, !direction);
  } else {
    digitalWrite(dirPinDischarge, direction);
  }

  if (direction != previousDirectionDischarge) {
    stepCountAccDischarge = 0;
    previousDirectionDischarge = direction;
  }
}

void calculateIntervalDischarge() {
  float errorDischarge = fabs(currentPressureDischarge - setPressureDischarge);
  // Accelerate towards max speed
  if (stepCountAccDischarge < accelerationStepsDischarge) {
    float progress = (float)stepCountAccDischarge / accelerationStepsDischarge;
    intervalDischarge = initialIntervalDischarge - (progress * (initialIntervalDischarge - intervalRunDischarge));
    stepCountAccDischarge++;
  }

  if (errorDischarge < 0.2 && setPressureDischarge != 0) {  // Check if within 0.2 bar of target pressure
    intervalDischarge = intervalRunDischarge * 4.0;         // Slow down (double the interval)
  } else {
    intervalDischarge = intervalRunDischarge;  // Normal speed
  }
}

void moveMotorDischarge(bool direction) {
  setMotorDirectionDischarge(direction);
  calculateIntervalDischarge();
  digitalWrite(stepPinDischarge, HIGH);
  delayMicroseconds(2);
  digitalWrite(stepPinDischarge, LOW);
  motorLastStepTimeDischarge = (int32_t)micros();
  stepCountDischarge += direction ? 1 : -1;
  if (notifyDischarge == true) {
    Serial.println("adjusting discharge...");
    notifyDischarge = false;
  }
}

void resetMotorDischarge() {
  if (currentPressureDischarge > minimumPressureDischarge) {
    moveMotorDischarge(false);
  } else {
    reset = false;
    stepCountDischarge = 0;
    stepCountAccDischarge = 0;
    setPressureDischarge = 0;
    Serial.println("discharge reset complete");
  }
}

// Pressure sensor function
void pressureSensor(CircularBuffer &buffer, int analogPin, volatile float &currentPressure, bool isInlet) {
  if ((int32_t)micros() - ticktime >= avgSampTime) {
    adc_select_input(analogPin);  // Select ADC input pin
    sleep_us(10);                 // The ADC needs a bit of time to stabilize after switching channels:
    int rawAnalog = adc_read();
    buffer.push(rawAnalog);
    float avgReading = buffer.average();
    if (isInlet) {
      currentPressure = avgReading * INLET_PRESSURE_CONV + INLET_SENSOR_OFFSET;
    } else {
      currentPressure = avgReading * DISCHARGE_PRESSURE_CONV + DISCHARGE_SENSOR_OFFSET;
    }
    ticktime = (int32_t)micros();
  }
}

void printBuffer(CircularBuffer &cb, const char *label) {
  Serial.printf("%s buffer [oldest to newest]:\n", label);

  for (int i = 0; i < cb.count; i++) {
    int idx = (cb.start + i) % arraySize;
    Serial.printf("%d", cb.buffer[idx]);
    if (i < cb.count - 1) Serial.print(", ");
  }

  Serial.println();  // Newline at end
}

void serial() {
  recvWithEndMarker();
  showNewNumber();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  if (Serial.available() > 0 && !newData) {
    rc = Serial.read();
    if (rc != endMarker) {
      receivedChars[ndx++] = rc;
      if (ndx >= numChars) ndx = numChars - 1;
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }
}

void showNewNumber() {
  if (newData) {
    processSerialInput();
    Serial.println();
    newData = false;
  }
}

void processSerialInput() {
  char *commaPosition = strchr(receivedChars, ',');  // Find comma position
  if (commaPosition != nullptr) {
    *commaPosition = '\0';  // Split the string
    float inletPressure = atof(receivedChars);
    float dischargePressure = atof(commaPosition + 1);

    // Ensure inletPressure is within [-1.0, 0.0]
    if (inletPressure < -1.0) {
      Serial.println("Warning: Inlet pressure below minimum (-1.0 bar). Adjusting to 0.0 bar.");
      inletPressure = 0.0;
    } else if (inletPressure > 0.0) {
      Serial.println("Warning: Inlet pressure above maximum (0.0 bar). Adjusting to 0.0 bar.");
      inletPressure = 0.0;
    }

    // Ensure dischargePressure is within [0.0, 7.0]
    if (dischargePressure < 0.0) {
      Serial.println("Warning: Discharge pressure below minimum (0.0 bar). Adjusting to 0.0 bar.");
      dischargePressure = 0.0;
    } else if (dischargePressure > 7.0) {
      Serial.println("Warning: Discharge pressure above maximum (7.0 bar). Adjusting to 0.0 bar.");
      dischargePressure = 0.0;
    }

    // Assign validated values
    setPressureInlet = inletPressure;
    setPressureDischarge = dischargePressure;

    Serial.printf("Set Pressures: Inlet = %.2f bar, Discharge = %.2f bar\n", setPressureInlet, setPressureDischarge);
    notifyInlet = false;
    notifyDischarge = false;
  } else {
    dataNumber = atof(receivedChars);
    processCommand();
  }
}

const char *Help_Message[] = {
  "=== COMMAND LIST ===",
  "start                  - Enable pressure control",
  "stop                   - Disable pressure control",
  "reset                  - Reset both motors to 0 steps",
  "steps                  - Display current step count and pressure readings",
  "back_inlet             - Move inlet motor backwards (increase step count)",
  "back_discharge         - Move discharge motor backwards (increase step count)",
  "maxStepsInlet:<value>  - Set max step count for inlet motor",
  "maxStepsDischarge:<value> - Set max step count for discharge motor",
  "config                 - Show current configuration values",
  "help                   - Show this help message",
  "======================",
  "=== INPUT FORMAT ===",
  "Inlet and Discharge Pressures: '<inlet_pressure>,<discharge_pressure>' (e.g., -0.5, 1.0)",
};

void printHelpMessage() {
  for (size_t i = 0; i < sizeof(Help_Message) / sizeof(Help_Message[0]); i++) {
    Serial.println(Help_Message[i]);  // Print each string in the array
  }
}


void processCommand() {
  serialStr = String(receivedChars);
  serialStr.trim();  // Remove delimiters

  if (serialStr == "stop") {
    Serial.println(" OK! Motors Disabled");
    controlEnableInlet = false;
    controlEnableDischarge = false;
    reset = false;

  } else if (serialStr == "start") {
    Serial.println(" OK! Motors Enabled");
    controlEnableInlet = true;
    controlEnableDischarge = true;
    reset = false;

  } else if (serialStr.startsWith("maxStepsInlet")) {
    maxStepsInlet = serialStr.substring(13).toInt();
    Serial.print(" OK! Inlet max steps set to ");
    Serial.println(maxStepsInlet);

  } else if (serialStr.startsWith("maxStepsDischarge")) {
    maxStepsDischarge = serialStr.substring(17).toInt();
    Serial.print(" OK! Discharge max steps set to ");
    Serial.println(maxStepsDischarge);

  } else if (serialStr == "back_inlet") {
    Serial.println(" OK! Move inlet motor backwards");
    stepCountInlet += 2500;

  } else if (serialStr == "back_discharge") {
    Serial.println(" OK! Move discharge motor backwards");
    stepCountDischarge += 2500;

  } else if (serialStr == "steps") {
    Serial.printf(" Inlet Steps: %ld\tDischarge Steps: %ld\n", stepCountInlet, stepCountDischarge);
    Serial.printf(" Inlet Pressure: %.2f bar\tDischarge Pressure: %.2f bar\n", currentPressureInlet, currentPressureDischarge);
    Serial.printf("Inlet motor enabled: %s\tDischarge motor enabled: %s\n",
                  controlEnableInlet ? "true" : "false",
                  controlEnableDischarge ? "true" : "false");


  } else if (serialStr == "reset") {
    Serial.println(" Resetting both motors...");
    reset = true;

  } else if (serialStr == "config") {
    Serial.println("==== CONFIG ====");
    Serial.printf("Max Steps - Inlet: %ld, Discharge: %ld\n", maxStepsInlet, maxStepsDischarge);
    Serial.printf("Accel Steps - Inlet: %ld, Discharge: %ld\n", accelerationStepsInlet, accelerationStepsDischarge);
    Serial.printf("Initial Interval: %ld, Run Interval: %ld\n", initialIntervalInlet, intervalRunInlet);
    Serial.printf("Initial Interval: %ld, Run Interval: %ld\n", initialIntervalDischarge, intervalRunDischarge);
    Serial.printf("Inlet Offset: %.2f, Discharge Offset: %.2f\n", INLET_SENSOR_OFFSET, DISCHARGE_SENSOR_OFFSET);
    Serial.println("================");
  } else if (serialStr == "help") {
    printHelpMessage();  // Call the function to print the help message
  } else if (serialStr == "stream") {
    streamPressure = true;
    Serial.println("Pressure streaming started");
  } else if (serialStr == "stop_stream") {
    streamPressure = false;
    Serial.println("Pressure streaming stopped");
  } else {
    Serial.println("Unknown command. Type 'help' for available options.");
  }
}
