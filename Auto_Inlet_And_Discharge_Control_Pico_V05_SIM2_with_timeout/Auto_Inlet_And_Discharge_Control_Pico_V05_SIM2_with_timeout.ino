// Raspberry Pi Pico and TB6600 stepper driver set to 1A.
// Nema17 stepper with 51:1 gearbox, driven with 1/8 microstepPinInletg.

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "pico/rand.h"

// === Global State Variables ===
unsigned long ticktime = 0;
int top = -1;  // Stack is initially empty
bool reset = false;

// === ADC & Sensor Constants ===
const float MY_ADC_RESOLUTION = 4095.0;

// Inlet Sensor
const float INLET_SENSOR_SCALE = 0.99;
const float INLET_SENSOR_RANGE = 2.0;  // -1 to +1 bar
const float INLET_SENSOR_OFFSET = 0.0f;
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
long maxStepsInlet = 32500;
long maxStepsDischarge = 35000;

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



// === Timeout Handling ===
unsigned long inletStartTime = 0;
bool inletTargetReached = false;

unsigned long dischargeStartTime = 0;
bool dischargeTargetReached = false;

const unsigned long timeoutDuration = 60000; // 60 seconds
// === Overshoot Correction Parameters ===
int reverseStepsInlet = 20;
float slowFactorInlet = 0.5;
bool recoveringInlet = false;

int reverseStepsDischarge = 20;
float slowFactorDischarge = 0.5;
bool recoveringDischarge = false;
// === Pressure Variables ===
volatile float currentPressureInlet = 0.0;
volatile float currentPressureDischarge = 0.0;

float setPressureInlet = 0.0;
float setPressureDischarge = 0.0;
float avg = 0.0;
float avgVoltage = 0.0;
float pressureToleranceInlet = 0.02;
float pressureToleranceDischarge = 0.05;
float maxPressureInlet = 0.0;
float minimumPressureInlet = -1.0;
float maxPressureDischarge = 7.0;
float minimumPressureDischarge = 0.2;
const float PRESSURE_RESPONSE_RATE = 0.05;  // Lower = slower response (e.g., 0.01 to 0.1)
const float INLET_PRESSURE_PER_STEP = minimumPressureInlet / maxStepsInlet;
bool targetPressure = false;
bool targetDischargePressure = false;
bool notifyInlet = false;
bool notifyDischarge = false;

bool controlEnableInlet = true;
bool controlEnableDischarge = true;

// === Circular Buffer ===
const int arraySize = 20;
int avgSampTime = 1000000 / arraySize;  // Sampling time in microseconds

// === Serial Communication ===
const byte numChars = 16;
char receivedChars[numChars];
boolean newData = false;
float dataNumber = 0;
String serialStr;

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

CircularBuffer dischargeFilo;      // Circular array for discharge pressure sensor
CircularBuffer inletFilo;          // Circular array for inlet pressure sensor
CircularBuffer flowFilo;           // Circular array for simulated flow sensor
volatile float currentFlow = 0.0;  // Current simulated flow value

void pressureSensorCore1();  // Core 1 Function Declaration

void setup() {
  pinMode(stepPinInlet, OUTPUT);
  pinMode(dirPinInlet, OUTPUT);
  pinMode(enaPinInlet, OUTPUT);
  pinMode(stepPinDischarge, OUTPUT);
  pinMode(dirPinDischarge, OUTPUT);
  pinMode(enaPinDischarge, OUTPUT);
  Serial.begin(115200);
  digitalWrite(stepPinInlet, LOW);
  digitalWrite(stepPinDischarge, LOW);
  // adc_init();         // Initialize ADC pins
  // adc_gpio_init(26);  // A0 (GPIO26)
  // adc_gpio_init(27);  // A1 (GPIO27)
  multicore_launch_core1(pressureSensorCore1);
  while (!Serial)
    ;
  Serial.println("Inlet and Discharge Pressure Control Software Version 0.01");
  Serial.println("Enter 'help' to list available serial commands");
}

void loop() {
  serial();  // Ensure serial input is processed first
  pressureControlInlet();
  pressureControlDischarge();
  if (streamPressure && millis() - lastStreamTime >= streamInterval) {
    lastStreamTime = millis();

    printBuffer(inletFilo, "Inlet", true);
    printBuffer(dischargeFilo, "Discharge", false);
    printBuffer(flowFilo, "Flow", false);
    delay(10);
  }
}

float simulateFlowSensor(float inletPressure, float dischargePressure) {
  const float BASE_FLOW = 1.5;
  const float k_discharge = 0.0714;  // L/min per bar
  const float k_inlet = 2.0;         // L/min per bar

  float flow = BASE_FLOW - k_discharge * dischargePressure + k_inlet * inletPressure;

  // Clamp flow to valid range
  if (flow < 0) flow = 0;
  return flow;
}


void pressureSensorCore1() {
  while (true) {
    // Simulated target pressure based on step count
    float targetPressureInletSim = stepCountInlet * INLET_PRESSURE_PER_STEP;

    // Exponential smoothing to simulate lag
    currentPressureInlet += (targetPressureInletSim - currentPressureInlet) * PRESSURE_RESPONSE_RATE;

    // Same logic for discharge
    float targetPressureDischargeSim = stepCountDischarge * (maxPressureDischarge / maxStepsDischarge);
    currentPressureDischarge += (targetPressureDischargeSim - currentPressureDischarge) * PRESSURE_RESPONSE_RATE;

    // Push to circular buffers
    inletFilo.push((int)(currentPressureInlet * 100));
    dischargeFilo.push((int)(currentPressureDischarge * 100));

    // Simulate flow as pressure difference × scaling
    float flowReading = simulateFlowSensor(currentPressureInlet, currentPressureDischarge);
    flowFilo.push((int)(flowReading * 100.0));
    currentFlow = flowFilo.average() / 100.0;

    sleep_us(avgSampTime);  // pacing if needed
  }
}




void pressureControlInlet() {
  digitalWrite(enaPinInlet, targetPressure ? HIGH : LOW);
  // Check if zero or negative target pressure is achieved
  if (!reset && setPressureInlet < 0 && fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet) {
    targetPressure = true;
    if (!notifyInlet) {
      Serial.printf("Inlet target pressure %.2f bar achieved\n", currentPressureInlet);
      notifyInlet = true;
    }
  }
  // Check if zero position is achieved
  else if (setPressureInlet == 0 && fabs(currentPressureInlet - setPressureInlet) >= maxPressureInlet && stepCountInlet == 0 && !reset) {
    targetPressure = true;
    if (!notifyInlet) {
      Serial.println("Inlet zero position achieved");
      notifyInlet = true;
    }
  } else {  // If neither condition is met
    targetPressure = false;
  }
  if ((int32_t)micros() - motorLastStepTime >= intervalInlet && !targetPressure && controlEnableInlet && !reset) {  // Adjust pressure if needed
    adjustPressureInlet();
  }
}

void pressureControlDischarge() {
  digitalWrite(enaPinDischarge, targetDischargePressure ? HIGH : LOW);
  // Check if the target discharge pressure is achieved
  if (!reset && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge) {
    targetDischargePressure = true;
    if (!notifyDischarge) {
      Serial.printf("Discharge target pressure %.2f bar achieved\n", currentPressureDischarge);
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
  } else {  // Otherwise, target pressure is not achieved
    targetDischargePressure = false;
  }
  if ((int32_t)micros() - motorLastStepTimeDischarge >= intervalDischarge && !targetDischargePressure && controlEnableDischarge && !reset) {  // Adjust pressure if needed
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

void setMotorDirectionInlet(bool direction) {  // Function to set motor direction based on motorDirectionInlet value
  if (motorDirectionInlet) {
    digitalWrite(dirPinInlet, !direction);  // Invert direction if motorDirectionInlet is true
  } else {
    digitalWrite(dirPinInlet, direction);  // Otherwise, use the direction as is
  }
  if (direction != previousDirectionInlet) {  // Check if the direction has changed
    stepCountAccInlet = 0;                    // Reset stepCountAcc to zero on direction change
    previousDirectionInlet = direction;       // Update the previous direction
  }
}


void calculateIntervalInlet() {
  float errorInlet = fabs(currentPressureInlet - setPressureInlet);
  if (stepCountAccInlet < accelerationStepsInlet) {
    float progress = (float)stepCountAccInlet / accelerationStepsInlet;
    intervalInlet = initialIntervalInlet - (progress * (initialIntervalInlet - intervalRunInlet));
    stepCountAccInlet++;
  }

  if (errorInlet < 0.2 && setPressureInlet != 0 || currentPressureInlet <= -0.25) {  // Check if within 0.1 bar of target pressure
    intervalInlet = intervalRunInlet * 16.0;                                         // Slow down (double the interval)
  } else if (errorInlet < 0.15 && setPressureInlet != 0) {
    intervalInlet = intervalRunInlet * 40.0;
  } else {
    intervalInlet = intervalRunInlet;
  }
}

void moveMotorInlet(bool direction) {  // Updated moveMotor function with acceleration
  if (controlEnableInlet) {
    setMotorDirectionInlet(direction);  // Use setmotorDirectionInlet to control dirPinInlet
    calculateIntervalInlet();           // Update interval based on step count for acceleration
    digitalWrite(stepPinInlet, HIGH);
    delayMicroseconds(5);
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
  if (currentPressureInlet > maxPressureInlet) {
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
  delayMicroseconds(5);
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

// void pressureSensor(CircularBuffer &buffer, int analogPin, volatile float &currentPressure, bool isInlet) {  //Simulated Pressure Sensor
//   static float simPressureInlet = 0.0;
//   static float simPressureDischarge = 0.0;
//   static unsigned long lastUpdateTime = 0;

//   if ((int32_t)micros() - ticktime >= avgSampTime) {
//     ticktime = (int32_t)micros();

//     // Generate random float in range [-0.05, 0.05]
//     float fluctuation = ((int32_t)(get_rand_32() & 0xFFFF) - 32768) / 65536.0f * 0.05f;  // ±0.05

//     if (isInlet) {
//       // Simulated pressure is proportional to steps
//       simPressureInlet += 0.00002f * (stepCountInlet - simPressureInlet * 10000.0f);  // Smooth transition
//       currentPressure = simPressureInlet + fluctuation;  // Add small random fluctuation
//       inletFilo.push(currentPressure * 1000);  // Scale for circular buffer
//     } else {
//       simPressureDischarge += 0.00001f * (stepCountDischarge - simPressureDischarge * 10000.0f);
//       currentPressure = simPressureDischarge + fluctuation;
//       dischargeFilo.push(currentPressure * 1000);
//     }
//   }
// }

void printBuffer(CircularBuffer &cb, const char *label, bool isInlet) {
  Serial.printf("%s", label);  // Start with just "INLET" or "DISCHARGE"

  for (int i = 0; i < cb.count; i++) {
    int idx = (cb.start + i) % arraySize;
    float rawValue = cb.buffer[idx];

    float pressure;
    if (isInlet) {
      float actualPressureInBar = rawValue / 100.0f;
      pressure = rawValue / 100.0f; /*actualPressureInBar + INLET_SENSOR_OFFSET;*/
    } else {
      pressure = rawValue / 100.0f; /* * DISCHARGE_PRESSURE_CONV + DISCHARGE_SENSOR_OFFSET;*/
    }

    Serial.printf(",%.2f", pressure);  // Add comma before each value
  }

  Serial.println();  // End line
  delay(100);
}

void serial() {
  recvWithEndMarker();
  showNewNumber();
}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  if (Serial.available() > 0) {
    rc = Serial.read();

    if (rc != endMarker) {
      if (ndx < numChars - 1) {  // Ensure space for null terminator
        receivedChars[ndx++] = rc;
      }
    } else {
      receivedChars[ndx] = '\0';  // Terminate string
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


void controlInletPressure() {
    if (!inletTargetReached) {
        if (millis() - inletStartTime > timeoutDuration) {
            Serial.println("Inlet pressure control timeout");
            inletTargetReached = true; // Stop further messages
        }
        if (abs(setPressureInlet - currentPressureInlet) <= pressureToleranceInlet) {
            inletTargetReached = true; // Target reached successfully
        }
    }
    
    float error = setPressureInlet - currentPressureInlet;

    if (abs(error) > pressureToleranceInlet) {
        if (!recoveringInlet) {
            // Overshoot detected - reverse briefly
            digitalWrite(dirPinInlet, !motorDirectionInlet);
            for (int i = 0; i < reverseStepsInlet; i++) {
                digitalWrite(stepPinInlet, HIGH);
                sleep_us(200);
                digitalWrite(stepPinInlet, LOW);
                sleep_us(200);
            }
            recoveringInlet = true;
        } else {
            // Re-approach slowly
            digitalWrite(dirPinInlet, error > 0 ? HIGH : LOW);
            for (int i = 0; i < 1; i++) {
                digitalWrite(stepPinInlet, HIGH);
                sleep_us((int)(intervalRunInlet / slowFactorInlet));
                digitalWrite(stepPinInlet, LOW);
                sleep_us((int)(intervalRunInlet / slowFactorInlet));
            }
            if (abs(error) <= pressureToleranceInlet / 2) {
                recoveringInlet = false;
            }
        }
    }
}


void controlDischargePressure() {
    if (!dischargeTargetReached) {
        if (millis() - dischargeStartTime > timeoutDuration) {
            Serial.println("Discharge pressure control timeout");
            dischargeTargetReached = true; // Stop further messages
        }
        if (abs(setPressureDischarge - currentPressureDischarge) <= pressureToleranceDischarge) {
            dischargeTargetReached = true; // Target reached successfully
        }
    }
    
    float error = setPressureDischarge - currentPressureDischarge;

    if (abs(error) > pressureToleranceDischarge) {
        if (!recoveringDischarge) {
            // Overshoot detected - reverse briefly
            digitalWrite(dirPinDischarge, !motorDirectionDischarge);
            for (int i = 0; i < reverseStepsDischarge; i++) {
                digitalWrite(stepPinDischarge, HIGH);
                sleep_us(200);
                digitalWrite(stepPinDischarge, LOW);
                sleep_us(200);
            }
            recoveringDischarge = true;
        } else {
            // Re-approach slowly
            digitalWrite(dirPinDischarge, error > 0 ? HIGH : LOW);
            for (int i = 0; i < 1; i++) {
                digitalWrite(stepPinDischarge, HIGH);
                sleep_us((int)(intervalRunDischarge / slowFactorDischarge));
                digitalWrite(stepPinDischarge, LOW);
                sleep_us((int)(intervalRunDischarge / slowFactorDischarge));
            }
            if (abs(error) <= pressureToleranceDischarge / 2) {
                recoveringDischarge = false;
            }
        }
    }
}
