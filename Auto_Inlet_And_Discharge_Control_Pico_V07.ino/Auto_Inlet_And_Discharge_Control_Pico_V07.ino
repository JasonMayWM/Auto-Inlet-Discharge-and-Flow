// Raspberry Pi Pico and TB6600 stepper driver set to 1A.
// Nema17 stepper with 51:1 gearbox, driven with 1/8 microstepping.
// This code controls two stepper motors (inlet and discharge) to regulate pressure.
// It uses a Raspberry Pi Pico, with one core handling motor control and serial commands,
// and the other core (core1) simulating pressure sensor readings.

#include "pico/stdlib.h"     // Standard library for Raspberry Pi Pico
#include "pico/multicore.h"  // For using both cores of RP2040
#include <stdio.h>           // Standard I/O functions (like printf)
#include "hardware/adc.h"    // For ADC (Analog to Digital Converter) functionalities
#include "pico/rand.h"       // For random number generation (if needed, currently not explicitly used for core logic)
// #include <cstring> // No longer needed as strcmp not used in printBuffer from your pasted code, and String _name was replaced by const char* earlier.

// === Global State Variables ===
unsigned long ticktime = 0;
int top = -1;
bool reset = false;

// === ADC & Sensor Constants ===
const float MY_ADC_RESOLUTION = 4095.0;

// Inlet Pressure Sensor Configuration
const float INLET_SENSOR_SCALE = 0.99;
const float INLET_SENSOR_RANGE = 2.0;
const float INLET_SENSOR_OFFSET = -1.00f;
const float INLET_PRESSURE_CONV = (INLET_SENSOR_RANGE * INLET_SENSOR_SCALE / MY_ADC_RESOLUTION);

// Discharge Pressure Sensor Configuration
const float DISCHARGE_SENSOR_SCALE = 1.0;
const float DISCHARGE_SENSOR_RANGE = 10.0;
const float DISCHARGE_SENSOR_OFFSET = -0.07f; // Your value from pasted code
const float DISCHARGE_PRESSURE_CONV = (DISCHARGE_SENSOR_RANGE * DISCHARGE_SENSOR_SCALE / MY_ADC_RESOLUTION);

// Flow Sensor Configuration
const float FLOW_SENSOR_SCALE = 1.0;
const float FLOW_SENSOR_RANGE = 3.0;   // Your value
const float FLOW_SENSOR_OFFSET = 0.0f;
const float FLOW_CONV = (FLOW_SENSOR_RANGE * FLOW_SENSOR_SCALE / MY_ADC_RESOLUTION);

enum SensorType {INLET, DISCHARGE, FLOW};

// === Pin Definitions ===
const int inletAnalogPin = 26;      // GPIO26, ADC0
const int dischargeAnalogPin = 27;  // GPIO27, ADC1
const int flowAnalogPin = 28;       // GPIO28, ADC2

const int stepPinInlet = 4;
const int dirPinInlet = 3;
const int enaPinInlet = 2;

const int stepPinDischarge = 8;
const int dirPinDischarge = 7;
const int enaPinDischarge = 6;

long maxStepsInlet = 325000;
long maxStepsDischarge = 350000;

long intervalRunInlet = 200;
long initialIntervalInlet = 1000;
long intervalRunDischarge = 200;
long initialIntervalDischarge = 1000;

long accelerationStepsInlet = 2000;
long accelerationStepsDischarge = 1500;

const unsigned long timeoutDuration = 180000;

// Pressure Variables
volatile float currentPressureInlet = 0.0;
volatile float currentPressureDischarge = 0.0;
float setPressureInlet = 0.0;
float setPressureDischarge = 0.0;

// Tolerances for target achievement (used in main loop)
float pressureToleranceInlet = 0.025f;
float pressureToleranceDischarge = 0.025f;
// Other pressure limits
float maxPressureInlet = 0.0f;
float minimumPressureInlet = -1.0f;
float maxPressureDischarge = 7.0f;
float minimumPressureDischarge = 0.2f;

// Global Flags for System State
bool targetPressure = false;
bool targetDischargePressure = false;
bool targetAchievedMsgPrintedInlet = false;
bool targetAchievedMsgPrintedDischarge = false;
bool controlEnableInlet = true;
bool controlEnableDischarge = true;

// Global parameters that were for overshoot (now unused by MotorController, can be removed if not used elsewhere)
// int reverseStepsInlet = 3000;
// float slowFactorInlet = 0.5f;
// int reverseStepsDischarge = 1000;
// float slowFactorDischarge = 0.5f;

// === Circular Buffer Configuration ===
const int arraySize = 250;
int avgSampTime = 1000000 / arraySize;

// === Serial Communication ===
const byte numChars = 16;
char receivedChars[numChars];
boolean newData = false;
String serialStr; // Still used in processCommand, keep for now unless you want to change all serial handling
bool streamPressure = false;
unsigned long lastStreamTime = 0;
const unsigned long streamInterval = 1000;

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

CircularBuffer dischargeFilo;
CircularBuffer inletFilo;
CircularBuffer flowFilo;
volatile float currentFlow = 0.0;

class MotorController {
public:
  int _stepPin;
  int _dirPin;
  int _enaPin;
  long _maxSteps;
  long _initialInterval;
  long _runInterval;
  long _accelerationSteps;
  bool _motorDirectionSetting;
  volatile float& _currentPressureRef;
  float& _setPressureRef;
  bool& _controlEnableRef;
  bool& _targetAchievedMsgPrintedFlagRef;
  float _minPressureForReset;
  const char* _name; // Changed from String to const char*
  bool _adjustingMsgPrinted;
  long _stepCount;
  long _stepCountAcc;
  bool _previousDirection;
  unsigned long _motorLastStepTime;
  long _currentInterval;
  float _slowdownPressureThreshold1;
  float _slowdownFactor1;
  float _slowdownPressureThreshold2;
  float _slowdownFactor2;
  unsigned long _pressureControlStartTime;
  bool _timedOut;
  // Overshoot members removed:
  // int _overshootReverseSteps;
  // float _overshootSlowFactor;
  // bool _isRecoveringOvershoot;
  // float _pressureTolerance;

  MotorController(int stepPin, int dirPin, int enaPin,
                  long maxSteps, long initialInterval, long runInterval, long accelerationSteps,
                  bool motorDirectionSetting,
                  volatile float& currentPressureRef, float& setPressureRef,
                  bool& controlEnableRef, bool& targetAchievedMsgPrintedFlagRef,
                  float minPressureForReset,
                  float slowdownPressureThreshold1, float slowdownFactor1,
                  float slowdownPressureThreshold2, float slowdownFactor2,
                  const char* name) // name is now const char*
    : _stepPin(stepPin), _dirPin(dirPin), _enaPin(enaPin),
      _maxSteps(maxSteps), _initialInterval(initialInterval), _runInterval(runInterval), _accelerationSteps(accelerationSteps),
      _motorDirectionSetting(motorDirectionSetting),
      _currentPressureRef(currentPressureRef), _setPressureRef(setPressureRef),
      _controlEnableRef(controlEnableRef), _targetAchievedMsgPrintedFlagRef(targetAchievedMsgPrintedFlagRef),
      _minPressureForReset(minPressureForReset),
      _slowdownPressureThreshold1(slowdownPressureThreshold1), _slowdownFactor1(slowdownFactor1),
      _slowdownPressureThreshold2(slowdownPressureThreshold2), _slowdownFactor2(slowdownFactor2),
      _name(name)
      // Overshoot initializers removed
  {
    _stepCount = 0;
    _stepCountAcc = 0;
    _previousDirection = true;
    _motorLastStepTime = 0;
    _currentInterval = _initialInterval;
    _pressureControlStartTime = 0;
    _timedOut = false;
    _adjustingMsgPrinted = false;

    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enaPin, OUTPUT);
    digitalWrite(_stepPin, LOW);
    digitalWrite(_enaPin, HIGH);
  }

  void setDirection(bool direction) {
    if (_motorDirectionSetting) {
      digitalWrite(_dirPin, !direction);
    } else {
      digitalWrite(_dirPin, direction);
    }
    if (direction != _previousDirection) {
      _stepCountAcc = 0;
      _previousDirection = direction;
    }
  }

  void enableMotor(bool enable) {
    digitalWrite(_enaPin, enable ? LOW : HIGH);
  }

  bool isControlEnabled() {
    return _controlEnableRef;
  }

  void setControlEnable(bool enable) {
    _controlEnableRef = enable;
  }

  long getStepCount() {
    return _stepCount;
  }

  long increaseStepCount() { // Your function
    _stepCount = _stepCount + 2500;
    return _stepCount;
  }

  long getMaxSteps() {
    return _maxSteps;
  }

  void setTargetAchievedMsgPrinted(bool val) {
    _targetAchievedMsgPrintedFlagRef = val;
  }

  bool getTargetAchievedMsgPrinted() const {
    return _targetAchievedMsgPrintedFlagRef;
  }

  // bool isRecoveringOvershoot() const { // Removed
  //   return _isRecoveringOvershoot;
  // }

  void calculateInterval() {
    // Overshoot recovery logic removed
    float error = fabs(_currentPressureRef - _setPressureRef);

    if (_stepCountAcc < _accelerationSteps) {
      float progress = (float)_stepCountAcc / _accelerationSteps;
      _currentInterval = _initialInterval - (progress * (_initialInterval - _runInterval));
      _stepCountAcc++;
    } else {
      _currentInterval = _runInterval;
    }

    if (_setPressureRef != 0) {
      if (_slowdownPressureThreshold2 != -1.0f && error < _slowdownPressureThreshold2) {
        _currentInterval = _runInterval * _slowdownFactor2;
      }
      else if (error < _slowdownPressureThreshold1) {
        _currentInterval = _runInterval * _slowdownFactor1;
      }
    }
    if (_currentInterval <= 0) { _currentInterval = _runInterval; }
  }

  void stepMotor(bool direction) {
    if (!_controlEnableRef) return;

    if ((int32_t)micros() - _motorLastStepTime < _currentInterval) { return; }

    setDirection(direction);
    calculateInterval();

    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(_stepPin, LOW);

    _motorLastStepTime = micros();
    _stepCount += direction ? 1 : -1;

    if (getTargetAchievedMsgPrinted() && !_adjustingMsgPrinted && _controlEnableRef && _setPressureRef != 0) {
      Serial.print(_name); // _name is now const char*
      Serial.println(" adjusting...");
      _adjustingMsgPrinted = true;
      setTargetAchievedMsgPrinted(false);
    }
  }

  bool resetMotor() {
    enableMotor(true);
    setControlEnable(true);
    Serial.print(_name); // _name is now const char*
    Serial.println(" motor resetting...");
    unsigned long resetOpStartTime = millis();
    const unsigned long resetOperationTimeout = 30000;

    while (millis() - resetOpStartTime < resetOperationTimeout) {
      bool conditionMet = (_currentPressureRef <= _minPressureForReset);
      if (conditionMet) {
        _stepCount = 0;
        _stepCountAcc = 0;
        _setPressureRef = 0;
        Serial.print(_name); // _name is now const char*
        Serial.println(" reset complete (pressure condition met).");
        return true;
      }

      if ((int32_t)micros() - _motorLastStepTime >= (_initialInterval > 500 ? 500 : _initialInterval)) {
        setDirection(false);
        _currentInterval = _initialInterval > 500 ? 500 : _initialInterval;
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(_stepPin, LOW);
        _motorLastStepTime = micros();
        _stepCount += -1;
      }
    }
    Serial.print(_name); // _name is now const char*
    Serial.println(" motor reset timeout!");
    enableMotor(false);
    return false;
  }

  unsigned long getLastStepTime() {
    return _motorLastStepTime;
  }

  long getCurrentInterval() {
    return _currentInterval;
  }

  void resetTimeout(float newSetPressure) {
    if (newSetPressure != 0) {
      _pressureControlStartTime = millis();
      _timedOut = false;
    } else {
      _pressureControlStartTime = 0;
      _timedOut = false;
    }
  }

  void managePressure(bool targetPressureFlag, bool globalResetFlag) {
    if (_timedOut) {
      enableMotor(false);
      return;
    }

    if (globalResetFlag) {
      if (resetMotor()) {
      }
      return;
    }

    enableMotor(!targetPressureFlag);

    if (!targetPressureFlag && _setPressureRef != 0 && isControlEnabled()) {
      if (_pressureControlStartTime != 0 && millis() - _pressureControlStartTime > timeoutDuration) {
        Serial.print(_name); // _name is now const char*
        Serial.println(" motor pressure control timeout!");
        _timedOut = true;
        setControlEnable(false);
        enableMotor(false);
        return;
      }
    }

    if (!isControlEnabled() || targetPressureFlag) { return; }

    // Overshoot logic removed from here

    if (_setPressureRef != 0 && _stepCount > 0 && ((_currentPressureRef > _setPressureRef && _setPressureRef > 0) || (_currentPressureRef < _setPressureRef && _setPressureRef < 0))) {
      stepMotor(false);
    }
    else if (_setPressureRef != 0 && _stepCount < _maxSteps && ((_currentPressureRef < _setPressureRef && _setPressureRef > 0) || (_currentPressureRef > _setPressureRef && _setPressureRef < 0))) {
      stepMotor(true);
    }
    else if (_stepCount >= _maxSteps && _setPressureRef != 0) {
      setControlEnable(false);
      Serial.print(_name); // _name is now const char*
      Serial.println(" motor at maximum step count. Motor disabled, reset pressure to '0' and then 'start' to enable.");
    }
    else if (_setPressureRef == 0 && _stepCount > 0) {
      stepMotor(false);
    }
  }
};

MotorController inletMotor(
  stepPinInlet, dirPinInlet, enaPinInlet,
  maxStepsInlet, initialIntervalInlet, intervalRunInlet, accelerationStepsInlet,
  true,
  currentPressureInlet, setPressureInlet, controlEnableInlet, targetAchievedMsgPrintedInlet,
  maxPressureInlet,
  0.2f, 16.0f, 0.15f, 40.0f,
  "Inlet" // Name is now const char*
  // Removed overshoot arguments
);

MotorController dischargeMotor(
  stepPinDischarge, dirPinDischarge, enaPinDischarge,
  maxStepsDischarge, initialIntervalDischarge, intervalRunDischarge, accelerationStepsDischarge,
  true,
  currentPressureDischarge, setPressureDischarge, controlEnableDischarge, targetAchievedMsgPrintedDischarge,
  minimumPressureDischarge,
  0.2f, 8.0f, 0.0f, 1.0f,
  "Discharge" // Name is now const char*
  // Removed overshoot arguments
);

void pressureSensorCore1();

void setup() {
  Serial.begin(115200);

  adc_init();
  adc_gpio_init(inletAnalogPin);   // Your code uses GPIO number directly
  adc_gpio_init(dischargeAnalogPin);
  adc_gpio_init(flowAnalogPin);

  multicore_launch_core1(pressureSensorCore1);

  while (!Serial)
    ;

  Serial.println("Inlet and Discharge Pressure Control Software Version 0.05 (MotorController Refactor)");
  Serial.println("Enter 'help' to list available serial commands");
}

// Full operational loop from your pasted code:
void loop() {
  serial();

  bool inletTargetAchieved = false;
  if (!reset) {
    if (setPressureInlet < 0) {
      if (fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet /* && !inletMotor.isRecoveringOvershoot() - Condition modified */) {
        inletTargetAchieved = true;
        if (!inletMotor.getTargetAchievedMsgPrinted()) {
          Serial.printf("Inlet target pressure %.2f bar achieved", currentPressureInlet);
          inletMotor.setTargetAchievedMsgPrinted(true);
          inletMotor._adjustingMsgPrinted = false;
        }
      }
    }
    else if (setPressureInlet == 0) {
      if (inletMotor.getStepCount() == 0  && fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet /* You had fabs(currentPressureInlet) <= pressureToleranceInlet; changed to match V04 more closely */ ) { // Adjusted to match V04 logic for 0 bar more closely
        inletTargetAchieved = true;
        if (!inletMotor.getTargetAchievedMsgPrinted()) {
          Serial.println("Inlet zero position achieved");
          inletMotor.setTargetAchievedMsgPrinted(true);
          inletMotor._adjustingMsgPrinted = false;
        }
      }
    }
    else {
      if (fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet /* && !inletMotor.isRecoveringOvershoot() - Condition modified */) {
        inletTargetAchieved = true;
        if (!inletMotor.getTargetAchievedMsgPrinted()) {
          Serial.printf("Inlet target pressure %.2f bar achieved", currentPressureInlet);
          inletMotor.setTargetAchievedMsgPrinted(true);
          inletMotor._adjustingMsgPrinted = false;
        }
      }
    }
  }
  targetPressure = inletTargetAchieved;
  inletMotor.managePressure(targetPressure, reset);

  bool dischargeTargetAchievedFull = false; // Renamed from targetDischargePressure to avoid conflict with global
  if (!reset && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge && setPressureDischarge != 0) {
    dischargeTargetAchievedFull = true;
    if (!dischargeMotor.getTargetAchievedMsgPrinted()) {
      Serial.printf("Discharge target pressure %.2f bar achieved", currentPressureDischarge);
      dischargeMotor.setTargetAchievedMsgPrinted(true);
      dischargeMotor._adjustingMsgPrinted = false;
    }
  } else if (setPressureDischarge == 0 && dischargeMotor.getStepCount() == 0 && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge && !reset) { // Adjusted to match V04 logic for 0 bar
    dischargeTargetAchievedFull = true;
    if (!dischargeMotor.getTargetAchievedMsgPrinted()) {
      Serial.println("Discharge zero position achieved");
      dischargeMotor.setTargetAchievedMsgPrinted(true);
      dischargeMotor._adjustingMsgPrinted = false;
    }
  } else {
    // dischargeTargetAchievedFull remains false
  }
  targetDischargePressure = dischargeTargetAchievedFull;
  dischargeMotor.managePressure(targetDischargePressure, reset);

  if (streamPressure && millis() - lastStreamTime >= streamInterval) {
    lastStreamTime = millis();
    printBuffer(inletFilo, "Inlet", INLET);
    printBuffer(dischargeFilo, "Discharge", DISCHARGE);
    printBuffer(flowFilo, "Flow", FLOW);
    delay(10);
  }
}

void pressureSensorCore1() {
  while (true) {
    adc_select_input(0); // Using channel number directly as per your fix
    sleep_us(10);
    uint16_t rawInlet = adc_read();
    inletFilo.push(rawInlet);
    currentPressureInlet = (inletFilo.average() * INLET_PRESSURE_CONV) + INLET_SENSOR_OFFSET;

    adc_select_input(1); // Using channel number directly
    sleep_us(10);
    uint16_t rawDischarge = adc_read();
    dischargeFilo.push(rawDischarge);
    currentPressureDischarge = (dischargeFilo.average() * DISCHARGE_PRESSURE_CONV) + DISCHARGE_SENSOR_OFFSET;

    adc_select_input(2); // Using channel number directly
    sleep_us(10);
    uint16_t rawFlow = adc_read();
    flowFilo.push(rawFlow);
    currentFlow = (flowFilo.average() * FLOW_CONV) + FLOW_SENSOR_OFFSET;

    sleep_us(avgSampTime);
  }
}

void printBuffer(CircularBuffer& cb, const char* label, SensorType type) {
  Serial.printf("%s", label);
  for (int i = 0; i < cb.count; i++) {
    int idx = (cb.start + i) % arraySize;
    uint16_t rawValue = cb.buffer[idx];
    float convertedValue;
    switch (type) {
      case INLET:
        convertedValue = (rawValue * INLET_PRESSURE_CONV) + INLET_SENSOR_OFFSET;
        break;
      case DISCHARGE:
        convertedValue = (rawValue * DISCHARGE_PRESSURE_CONV) + DISCHARGE_SENSOR_OFFSET;
        break;
      case FLOW:
        convertedValue = (rawValue * FLOW_CONV) + FLOW_SENSOR_OFFSET;
        break;
    }
    Serial.printf(",%.2f", convertedValue);
  }
  Serial.println();
  delay(100);
}

void serial() {
  recvWithEndMarker();
  showNewNumber();
}

void recvWithEndMarker() {
  static byte ndx = 0;    // Index for storing characters in receivedChars.
  char endMarker = '\n';  // Command terminator.
  char rc;                // Character read from serial.

  if (Serial.available() > 0) {  // If there's data in the serial buffer...
    rc = Serial.read();
    if (rc != endMarker) {          // If it's not the end marker...
      if (ndx < numChars - 1) {     // And there's space in the buffer...
        receivedChars[ndx++] = rc;  // Store the character.
      }
      // Else: buffer full, character discarded.
    } else {                      // If it IS the end marker...
      receivedChars[ndx] = '\0';  // Null-terminate the string.
      ndx = 0;                    // Reset index for next command.
      newData = true;             // Flag that new data is ready.
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
  char* commaPosition = strchr(receivedChars, ',');
  if (commaPosition != nullptr) {
    *commaPosition = '\0'; ;
    float newInletPressure = atof(receivedChars);
    float newDischargePressure = atof(commaPosition + 1);

    if (newInletPressure < -1.0f) newInletPressure = 0.0f;
    else if (newInletPressure > 0.0f) newInletPressure = 0.0f;

    if (newDischargePressure < 0.0f) newDischargePressure = 0.0f;
    else if (newDischargePressure > 7.0f) newDischargePressure = 0.0f;

    setPressureInlet = newInletPressure;
    setPressureDischarge = newDischargePressure;

    inletMotor.setTargetAchievedMsgPrinted(false);
    inletMotor._adjustingMsgPrinted = false;
    dischargeMotor.setTargetAchievedMsgPrinted(false);
    dischargeMotor._adjustingMsgPrinted = false;

    Serial.printf("Set Pressures: Inlet = %.2f bar, Discharge = %.2f bar", setPressureInlet, setPressureDischarge);

    if (setPressureInlet == 0) {
      // Condition from your code: if (inletMotor.getStepCount() == 0 && fabs(currentPressureInlet) <= pressureToleranceInlet)
      // My adjusted condition to remove isRecoveringOvershoot, and align with V04 which checks pressure against setPressure (0)
      if (inletMotor.getStepCount() == 0 && fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet) {
        Serial.println("Inlet zero position achieved");
        inletMotor.setTargetAchievedMsgPrinted(true);
      } else {
        Serial.println("Inlet adjusting...");
        inletMotor._adjustingMsgPrinted = true;
      }
    } else {
      if (fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet /* Condition was: && !inletMotor.isRecoveringOvershoot() */) {
        Serial.printf("Inlet target pressure %.2f bar achieved", currentPressureInlet);
        inletMotor.setTargetAchievedMsgPrinted(true);
      } else {
        Serial.println("Inlet adjusting...");
        inletMotor._adjustingMsgPrinted = true;
      }
    }

    if (setPressureDischarge == 0) {
      // Condition from your code: if (dischargeMotor.getStepCount() == 0 && fabs(currentPressureDischarge) <= pressureToleranceDischarge)
      // My adjusted condition:
      if (dischargeMotor.getStepCount() == 0 && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge) {
        Serial.println("Discharge zero position achieved");
        dischargeMotor.setTargetAchievedMsgPrinted(true);
      } else {
        Serial.println("Discharge adjusting...");
        dischargeMotor._adjustingMsgPrinted = true;
      }
    } else {
      if (fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge /* Condition was: && !dischargeMotor.isRecoveringOvershoot() */) {
        Serial.printf("Discharge target pressure %.2f bar achieved", currentPressureDischarge);
        dischargeMotor.setTargetAchievedMsgPrinted(true);
      } else {
        Serial.println("Discharge adjusting...");
        dischargeMotor._adjustingMsgPrinted = true;
      }
    }

    inletMotor.setControlEnable(true);
    controlEnableInlet = true;
    inletMotor.resetTimeout(setPressureInlet);
    dischargeMotor.setControlEnable(true);
    controlEnableDischarge = true;
    dischargeMotor.resetTimeout(setPressureDischarge);

  } else {
    processCommand();
  }
}

const char* Help_Message[] = {
  "=== COMMAND LIST ===",
  "start                  - Enable pressure control for both motors",
  "stop                   - Disable pressure control for both motors",
  "start_inlet            - Enable pressure control for inlet motor",
  "stop_inlet             - Disable pressure control for inlet motor",
  "start_discharge        - Enable pressure control for discharge motor",
  "stop_discharge         - Disable pressure control for discharge motor",
  "reset                  - Initiate reset for both motors (to pressure 0 for inlet, min pressure for discharge)",
  "steps                  - Display current step count and pressure readings",
  "maxStepsInlet:<value>  - Set max step count for inlet motor (Caution: not fully integrated with class yet)",
  "maxStepsDischarge:<value> - Set max step count for discharge motor (Caution: not fully integrated with class yet)",
  "config                 - Show current configuration values",
  "stream                 - Start streaming pressure data",
  "stop_stream            - Stop streaming pressure data",
  "help                   - Show this help message",
  "======================",
  "=== INPUT FORMAT ===",
  "Inlet and Discharge Pressures: '<inlet_pressure>,<discharge_pressure>' (e.g., -0.5,1.0)",
};

void printHelpMessage() {
  for (size_t i = 0; i < sizeof(Help_Message) / sizeof(Help_Message[0]); i++) {
    Serial.println(Help_Message[i]);
  }
}

void processCommand() {
  serialStr = String(receivedChars);
  serialStr.trim();

  if (serialStr == "stop") {
    Serial.println(" OK! Both Motors Disabled");
    inletMotor.setControlEnable(false);
    controlEnableInlet = false;
    dischargeMotor.setControlEnable(false);
    controlEnableDischarge = false;
    reset = false;
  } else if (serialStr == "start") {
    Serial.println(" OK! Both Motors Enabled");
    inletMotor.setControlEnable(true);
    controlEnableInlet = true;
    inletMotor.resetTimeout(setPressureInlet);
    inletMotor.setTargetAchievedMsgPrinted(false);
    targetAchievedMsgPrintedInlet = false;
    inletMotor._adjustingMsgPrinted = false;
    dischargeMotor.setControlEnable(true);
    controlEnableDischarge = true;
    dischargeMotor.resetTimeout(setPressureDischarge);
    dischargeMotor.setTargetAchievedMsgPrinted(false);
    targetAchievedMsgPrintedDischarge = false;
    dischargeMotor._adjustingMsgPrinted = false;
    reset = false;
  } else if (serialStr == "stop_inlet") {
    Serial.println(" OK! Inlet Motor Disabled");
    inletMotor.setControlEnable(false);
    controlEnableInlet = false;
  } else if (serialStr == "start_inlet") {
    Serial.println(" OK! Inlet Motor Enabled");
    inletMotor.setControlEnable(true);
    controlEnableInlet = true;
    inletMotor.resetTimeout(setPressureInlet);
    inletMotor.setTargetAchievedMsgPrinted(false);
    targetAchievedMsgPrintedInlet = false;
    inletMotor._adjustingMsgPrinted = false;
  } else if (serialStr == "stop_discharge") {
    Serial.println(" OK! Discharge Motor Disabled");
    dischargeMotor.setControlEnable(false);
    controlEnableDischarge = false;
  } else if (serialStr == "start_discharge") {
    Serial.println(" OK! Discharge Motor Enabled");
    dischargeMotor.setControlEnable(true);
    controlEnableDischarge = true;
    dischargeMotor.resetTimeout(setPressureDischarge);
    dischargeMotor.setTargetAchievedMsgPrinted(false);
    targetAchievedMsgPrintedDischarge = false;
    dischargeMotor._adjustingMsgPrinted = false;
  } else if (serialStr == "back_inlet") { // Your function
    Serial.println(" OK! Move inlet motor backwards");
    inletMotor.increaseStepCount();
  } else if (serialStr == "back_discharge") { // Your function
    Serial.println(" OK! Move discharge motor backwards");
    dischargeMotor.increaseStepCount();
  } else if (serialStr.startsWith("maxStepsInlet")) {
    long val = serialStr.substring(13).toInt();
    maxStepsInlet = val;
    Serial.print(" OK! Global maxStepsInlet set to ");
    Serial.println(maxStepsInlet);
    Serial.println(" Note: Restart or re-init motor object required for this to take full effect if not dynamically settable.");
  } else if (serialStr.startsWith("maxStepsDischarge")) {
    long val = serialStr.substring(17).toInt();
    maxStepsDischarge = val;
    Serial.print(" OK! Global maxStepsDischarge set to ");
    Serial.println(maxStepsDischarge);
    Serial.println(" Note: Restart or re-init motor object required for this to take full effect if not dynamically settable.");
  } else if (serialStr == "steps") {
    Serial.printf(" Inlet Steps: %ld (Current P: %.2f bar, Target P: %.2f bar)", inletMotor.getStepCount(), currentPressureInlet, setPressureInlet);
    Serial.printf(" Discharge Steps: %ld (Current P: %.2f bar, Target P: %.2f bar)", dischargeMotor.getStepCount(), currentPressureDischarge, setPressureDischarge);
    Serial.printf(" Flow rate: %.2f (L/min)", currentFlow);
    Serial.printf(" Inlet motor enabled: %s\tDischarge motor enabled: %s",inletMotor.isControlEnabled() ? "true" : "false", dischargeMotor.isControlEnabled() ? "true" : "false");
    Serial.printf(" Global reset flag: %s", reset ? "true" : "false");
    Serial.printf(" Inlet ADC Raw: %.2f, Discharge  ADC Raw: %.2f, Flow  ADC Raw: %.2f", inletFilo.average(), dischargeFilo.average(), flowFilo.average());
  } else if (serialStr == "reset") {
    Serial.println(" Initiating reset for both motors...");
    reset = true;
    Serial.println("Directly attempting Inlet motor reset sequence via managePressure...");
    inletMotor.managePressure(false, true);
    Serial.println("Directly attempting Discharge motor reset sequence via managePressure...");
    dischargeMotor.managePressure(false, true);
    Serial.println("Re-attempting reset via direct blocking calls:");
    bool inletResetDone = inletMotor.resetMotor();
    if (inletResetDone) {
      Serial.println("Inlet motor reset sequence reported success by direct call.");
    } else {
      Serial.println("Inlet motor reset sequence reported failure or timeout by direct call.");
    }
    bool dischargeResetDone = dischargeMotor.resetMotor();
    if (dischargeResetDone) {
      Serial.println("Discharge motor reset sequence reported success by direct call.");
    } else {
      Serial.println("Discharge motor reset sequence reported failure or timeout by direct call.");
    }
    if (inletResetDone && dischargeResetDone) {
      Serial.println("Both motors reported reset completion by direct calls.");
      reset = false;
    } else {
      Serial.println("One or both motors failed to reset via direct calls. Global 'reset' flag remains true.");
    }
  } else if (serialStr == "config") {
    Serial.println("==== CONFIG (via Motor Objects where applicable) ====");
    Serial.printf("Max Steps - Inlet: %ld, Discharge: %ld", inletMotor.getMaxSteps(), dischargeMotor.getMaxSteps());
    Serial.printf("Inlet Target Pressure for Reset: %.2f", inletMotor._minPressureForReset);
    Serial.printf("Discharge Target Pressure for Reset: %.2f", dischargeMotor._minPressureForReset);
    Serial.println("================");
  } else if (serialStr == "help") {
    printHelpMessage();
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