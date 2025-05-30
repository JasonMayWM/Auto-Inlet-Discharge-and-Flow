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

// Maximum steps (used for constructor)
long maxStepsInlet = 32500; 
long maxStepsDischarge = 35000;

// Step intervals (used for constructor)
long intervalRunInlet = 200; 
long initialIntervalInlet = 1000; 
long intervalRunDischarge = 200;  
long initialIntervalDischarge = 1000; 

// Acceleration (used for constructor)
long accelerationStepsInlet = 2000; 
long accelerationStepsDischarge = 1500; 

const unsigned long timeoutDuration = 60000; // 60 seconds (Global, used by MotorController)

// Pressure Variables
volatile float currentPressureInlet = 0.0;
volatile float currentPressureDischarge = 0.0;
float setPressureInlet = 0.0; 
float setPressureDischarge = 0.0; 

// Tolerances and limits (some will be passed to MotorController)
float pressureToleranceInlet = 0.02f; // Note: adding 'f' for float literal
float pressureToleranceDischarge = 0.05f;
float maxPressureInlet = 0.0f; 
float minimumPressureInlet = -1.0f;
float maxPressureDischarge = 7.0f;
float minimumPressureDischarge = 0.2f; 

const float PRESSURE_RESPONSE_RATE = 0.05f;  
const float INLET_PRESSURE_PER_STEP = minimumPressureInlet / maxStepsInlet; 

// Global flags
bool targetPressure = false; 
bool targetDischargePressure = false;
bool notifyInlet = false; 
bool notifyDischarge = false;
bool controlEnableInlet = true;
bool controlEnableDischarge = true;

// Global overshoot parameters (will be passed to MotorController constructor)
int reverseStepsInlet = 20;
float slowFactorInlet = 0.5f;
int reverseStepsDischarge = 20;
float slowFactorDischarge = 0.5f;

// === Circular Buffer ===
const int arraySize = 20;
int avgSampTime = 1000000 / arraySize;  

// === Serial Communication ===
const byte numChars = 16;
char receivedChars[numChars];
boolean newData = false;
String serialStr;
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
    bool& _notifyFlagRef;            
    float _minPressureForReset;   
    String _name;                 
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
    // Overshoot control members
    int _overshootReverseSteps;
    float _overshootSlowFactor;
    bool _isRecoveringOvershoot;
    float _pressureTolerance;

    MotorController(int stepPin, int dirPin, int enaPin,
                    long maxSteps, long initialInterval, long runInterval, long accelerationSteps,
                    bool motorDirectionSetting,
                    volatile float& currentPressureRef, float& setPressureRef, 
                    bool& controlEnableRef, bool& notifyFlagRef,
                    float minPressureForReset, 
                    float slowdownPressureThreshold1, float slowdownFactor1,
                    float slowdownPressureThreshold2, float slowdownFactor2,
                    String name,
                    int overshootReverseSteps,
                    float overshootSlowFactor,
                    float pressureToleranceVal) 
        : _stepPin(stepPin), _dirPin(dirPin), _enaPin(enaPin),
          _maxSteps(maxSteps), _initialInterval(initialInterval), _runInterval(runInterval), _accelerationSteps(accelerationSteps),
          _motorDirectionSetting(motorDirectionSetting),
          _currentPressureRef(currentPressureRef), _setPressureRef(setPressureRef), 
          _controlEnableRef(controlEnableRef), _notifyFlagRef(notifyFlagRef),
          _minPressureForReset(minPressureForReset), 
          _slowdownPressureThreshold1(slowdownPressureThreshold1), _slowdownFactor1(slowdownFactor1),
          _slowdownPressureThreshold2(slowdownPressureThreshold2), _slowdownFactor2(slowdownFactor2),
          _name(name),
          _overshootReverseSteps(overshootReverseSteps), _overshootSlowFactor(overshootSlowFactor),
          _pressureTolerance(pressureToleranceVal), _isRecoveringOvershoot(false)
    {
        _stepCount = 0;
        _stepCountAcc = 0;
        _previousDirection = true; 
        _motorLastStepTime = 0;
        _currentInterval = _initialInterval; 
        _pressureControlStartTime = 0; 
        _timedOut = false;             
        pinMode(_stepPin, OUTPUT);
        pinMode(_dirPin, OUTPUT);
        pinMode(_enaPin, OUTPUT);
        digitalWrite(_stepPin, LOW);
        digitalWrite(_enaPin, HIGH); 
    }

    void setDirection(bool direction) {
        if (_motorDirectionSetting) { digitalWrite(_dirPin, !direction); } 
        else { digitalWrite(_dirPin, direction); }
        if (direction != _previousDirection) { _stepCountAcc = 0; _previousDirection = direction; }
    }
    void enableMotor(bool enable) { digitalWrite(_enaPin, enable ? LOW : HIGH); }
    bool isControlEnabled() { return _controlEnableRef; }
    void setControlEnable(bool enable) { _controlEnableRef = enable; }
    long getStepCount() { return _stepCount; }
    long getMaxSteps() { return _maxSteps; }
    void setNotifyFlag(bool val) { _notifyFlagRef = val; }
    bool getNotifyFlag() { return _notifyFlagRef; }

    void calculateInterval() {
        if (_isRecoveringOvershoot) { // Check if recovering from overshoot first
            _currentInterval = (unsigned long)(_runInterval / _overshootSlowFactor);
            if (_currentInterval == 0) _currentInterval = 1; // Prevent issues
            return; // Skip normal accel/decel logic
        }
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
            } else if (error < _slowdownPressureThreshold1) {
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
        digitalWrite(_stepPin, HIGH); delayMicroseconds(5); digitalWrite(_stepPin, LOW);
        _motorLastStepTime = micros(); 
        _stepCount += direction ? 1 : -1;
        if (_notifyFlagRef == true && _name != "") { // Check name to prevent empty prints if name is not set
            Serial.print(_name); Serial.println(" adjusting...");
            // _notifyFlagRef = false; // Let managePressure or serialInput handle this based on overall target
        }
    }

    bool resetMotor() { 
        enableMotor(true); setControlEnable(true); 
        Serial.print(_name); Serial.println(" motor resetting...");
        unsigned long resetOpStartTime = millis(); 
        const unsigned long resetOperationTimeout = 30000; 
        while (millis() - resetOpStartTime < resetOperationTimeout) {
            bool conditionMet = (_currentPressureRef <= _minPressureForReset);
            if (conditionMet) {
                _stepCount = 0; _stepCountAcc = 0; _setPressureRef = 0; 
                Serial.print(_name); Serial.println(" reset complete (pressure condition met).");
                return true; 
            }
            if ((int32_t)micros() - _motorLastStepTime >= (_initialInterval > 500 ? 500 : _initialInterval)) {
                setDirection(false); 
                _currentInterval = _initialInterval > 500 ? 500 : _initialInterval; 
                digitalWrite(_stepPin, HIGH); delayMicroseconds(5); digitalWrite(_stepPin, LOW);
                _motorLastStepTime = micros(); _stepCount += -1; 
            }
        }
        Serial.print(_name); Serial.println(" motor reset timeout!");
        enableMotor(false); return false; 
    }
    
    unsigned long getLastStepTime() { return _motorLastStepTime; }
    long getCurrentInterval() { return _currentInterval; }

    void resetTimeout(float newSetPressure) {
        if (newSetPressure != 0) { _pressureControlStartTime = millis(); _timedOut = false; } 
        else { _pressureControlStartTime = 0; _timedOut = false; }
    }
    
    void managePressure(bool targetPressureFlag, bool globalResetFlag) {
        if (_timedOut) {
            enableMotor(false); return;
        }
        if (globalResetFlag) {
            if (resetMotor()) { /* Individual motor reset successful. */ }
            return; 
        }
        enableMotor(!targetPressureFlag); 
        if (!targetPressureFlag && _setPressureRef != 0 && isControlEnabled()) {
             if (_pressureControlStartTime != 0 && millis() - _pressureControlStartTime > timeoutDuration) { 
                Serial.print(_name); Serial.println(" motor pressure control timeout!");
                _timedOut = true; setControlEnable(false); enableMotor(false);    
                return; 
            }
        }
        if (!isControlEnabled() || targetPressureFlag) { return; }
        
        // Overshoot Logic (before normal stepping logic if not recovering)
        if (!_isRecoveringOvershoot) {
            bool overshootDetected = false;
            if ((_setPressureRef > 0 && _currentPressureRef > _setPressureRef + _pressureTolerance) ||
                (_setPressureRef < 0 && _currentPressureRef < _setPressureRef - _pressureTolerance)) {
                overshootDetected = true;
            }

            if (overshootDetected) {
                Serial.print(_name); Serial.println(" overshoot detected!");
                _isRecoveringOvershoot = true;
                // Determine actual pin state for reverse. _previousDirection true means last step was 'forward'
                bool reverseDirForPin = _previousDirection ? !_motorDirectionSetting : _motorDirectionSetting; // Corrected logic
                digitalWrite(_dirPin, reverseDirForPin); // Set reverse direction
                
                for (int i = 0; i < _overshootReverseSteps; i++) {
                    if(!isControlEnabled()) break; 
                    digitalWrite(_stepPin, HIGH); delayMicroseconds(200); 
                    digitalWrite(_stepPin, LOW); delayMicroseconds(200);
                    // Not updating _stepCount here for simplicity of recovery.
                }
                // After reversing, the next call to managePressure will enter the recovery state below.
                _motorLastStepTime = micros(); // Update last step time to allow recovery step soon
                return; 
            }
        } else { // _isRecoveringOvershoot is true
            bool directionToSetPoint = (_currentPressureRef < _setPressureRef);
            // calculateInterval will use the slow factor because _isRecoveringOvershoot is true
            stepMotor(directionToSetPoint); // stepMotor also calls calculateInterval which is now overshoot-aware

            if (fabs(_currentPressureRef - _setPressureRef) <= (_pressureTolerance / 2.0f)) {
                Serial.print(_name); Serial.println(" overshoot recovery complete.");
                _isRecoveringOvershoot = false;
            }
            return; // Return after a recovery step attempt
        }

        // Normal stepping logic (if no overshoot detected and not recovering)
        if (_setPressureRef != 0 && _stepCount > 0 && ((_currentPressureRef > _setPressureRef && _setPressureRef > 0) || (_currentPressureRef < _setPressureRef && _setPressureRef < 0))) {
            stepMotor(false); 
        } else if (_setPressureRef != 0 && _stepCount < _maxSteps && ((_currentPressureRef < _setPressureRef && _setPressureRef > 0) || (_currentPressureRef > _setPressureRef && _setPressureRef < 0))) {
            stepMotor(true);  
        } else if (_stepCount >= _maxSteps && _setPressureRef != 0) {
            setControlEnable(false);  
            Serial.print(_name); Serial.println(" motor at maximum step count. Motor disabled, reset pressure to '0' and then 'start' to enable.");
        } else if (_setPressureRef == 0 && _stepCount > 0) {
            stepMotor(false);  
        }
    }
};

MotorController inletMotor(
    stepPinInlet, dirPinInlet, enaPinInlet, maxStepsInlet, initialIntervalInlet, intervalRunInlet, accelerationStepsInlet, true, 
    currentPressureInlet, setPressureInlet, controlEnableInlet, notifyInlet, maxPressureInlet, 
    0.2f, 16.0f, 0.15f, 40.0f, "Inlet",
    reverseStepsInlet, slowFactorInlet, pressureToleranceInlet
);

MotorController dischargeMotor(
    stepPinDischarge, dirPinDischarge, enaPinDischarge, maxStepsDischarge, initialIntervalDischarge, intervalRunDischarge, accelerationStepsDischarge, true, 
    currentPressureDischarge, setPressureDischarge, controlEnableDischarge, notifyDischarge, minimumPressureDischarge, 
    0.2f, 4.0f, 0.0f, 1.0f, "Discharge", // Using 1.0f for factor2 if threshold2 is 0.0f
    reverseStepsDischarge, slowFactorDischarge, pressureToleranceDischarge
);

void pressureSensorCore1();
void setup() {
  Serial.begin(115200);
  multicore_launch_core1(pressureSensorCore1);
  while (!Serial);
  Serial.println("Inlet and Discharge Pressure Control Software Version 0.05 (MotorController Refactor)");
  Serial.println("Enter 'help' to list available serial commands");
}

void loop() {
  serial();
  if (!reset && setPressureInlet < 0 && fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet) {
      targetPressure = true;
      if (!inletMotor.getNotifyFlag()) { 
          Serial.printf("Inlet target pressure %.2f bar achieved\n", currentPressureInlet);
          inletMotor.setNotifyFlag(true);
      }
  } else if (setPressureInlet == 0 && fabs(currentPressureInlet) <= pressureToleranceInlet && inletMotor.getStepCount() == 0 && !reset) { 
      targetPressure = true;
      if (!inletMotor.getNotifyFlag()) {
          Serial.println("Inlet zero position achieved");
          inletMotor.setNotifyFlag(true);
      }
  } else if (setPressureInlet != 0 && fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet && !reset) { 
      targetPressure = true;
      if (!inletMotor.getNotifyFlag()) {
          Serial.printf("Inlet target pressure %.2f bar achieved\n", currentPressureInlet);
          inletMotor.setNotifyFlag(true);
      }
  } else { targetPressure = false; }
  inletMotor.managePressure(targetPressure, reset);

  if (!reset && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge && setPressureDischarge !=0) { 
      targetDischargePressure = true;
      if (!dischargeMotor.getNotifyFlag()) {
          Serial.printf("Discharge target pressure %.2f bar achieved\n", currentPressureDischarge);
          dischargeMotor.setNotifyFlag(true);
      }
  } else if (setPressureDischarge == 0 && fabs(currentPressureDischarge) <= pressureToleranceDischarge && dischargeMotor.getStepCount() == 0 && !reset) { 
      targetDischargePressure = true;
      if (!dischargeMotor.getNotifyFlag()) {
          Serial.println("Discharge zero position achieved");
          dischargeMotor.setNotifyFlag(true);
      }
  } else { targetDischargePressure = false; }
  dischargeMotor.managePressure(targetDischargePressure, reset);

  if (streamPressure && millis() - lastStreamTime >= streamInterval) {
    lastStreamTime = millis();
    printBuffer(inletFilo, "Inlet", true);
    printBuffer(dischargeFilo, "Discharge", false);
    printBuffer(flowFilo, "Flow", false);
    delay(10);
  }
}

float simulateFlowSensor(float inletPressure, float dischargePressure) {
  const float BASE_FLOW = 1.5f; const float k_discharge = 0.0714f; const float k_inlet = 2.0f;        
  float flow = BASE_FLOW - k_discharge * dischargePressure + k_inlet * inletPressure;
  if (flow < 0) flow = 0; return flow;
}
void pressureSensorCore1() {
  while (true) {
    float targetPressureInletSim = inletMotor.getStepCount() * INLET_PRESSURE_PER_STEP;
    currentPressureInlet += (targetPressureInletSim - currentPressureInlet) * PRESSURE_RESPONSE_RATE;
    float targetPressureDischargeSim = dischargeMotor.getStepCount() * (maxPressureDischarge / (dischargeMotor.getMaxSteps() == 0 ? 1 : dischargeMotor.getMaxSteps())); 
    currentPressureDischarge += (targetPressureDischargeSim - currentPressureDischarge) * PRESSURE_RESPONSE_RATE;
    inletFilo.push((int)(currentPressureInlet * 100)); dischargeFilo.push((int)(currentPressureDischarge * 100));
    float flowReading = simulateFlowSensor(currentPressureInlet, currentPressureDischarge);
    flowFilo.push((int)(flowReading * 100.0f)); currentFlow = flowFilo.average() / 100.0f;
    sleep_us(avgSampTime);
  }
}
void printBuffer(CircularBuffer &cb, const char *label, bool isInlet) {
  Serial.printf("%s", label); 
  for (int i = 0; i < cb.count; i++) {
    int idx = (cb.start + i) % arraySize; float rawValue = cb.buffer[idx];
    float pressure = rawValue / 100.0f; Serial.printf(",%.2f", pressure);
  }
  Serial.println(); delay(100);
}
void serial() { recvWithEndMarker(); showNewNumber(); }
void recvWithEndMarker() {
  static byte ndx = 0; char endMarker = '\n'; char rc;
  if (Serial.available() > 0) {
    rc = Serial.read();
    if (rc != endMarker) { if (ndx < numChars - 1) { receivedChars[ndx++] = rc; } } 
    else { receivedChars[ndx] = '\0'; ndx = 0; newData = true; }
  }
}
void showNewNumber() { if (newData) { processSerialInput(); Serial.println(); newData = false; } }

void processSerialInput() {
  char *commaPosition = strchr(receivedChars, ',');
  if (commaPosition != nullptr) {
    *commaPosition = '\0'; float newInletPressure = atof(receivedChars); float newDischargePressure = atof(commaPosition + 1);
    if (newInletPressure < -1.0f) newInletPressure = 0.0f; else if (newInletPressure > 0.0f) newInletPressure = 0.0f;
    if (newDischargePressure < 0.0f) newDischargePressure = 0.0f; else if (newDischargePressure > 7.0f) newDischargePressure = 0.0f;
    setPressureInlet = newInletPressure; setPressureDischarge = newDischargePressure;
    inletMotor.setNotifyFlag(false); notifyInlet = false; 
    inletMotor.resetTimeout(setPressureInlet);
    if (setPressureInlet != 0) { inletMotor.setControlEnable(true); controlEnableInlet = true; }
    dischargeMotor.setNotifyFlag(false); notifyDischarge = false; 
    dischargeMotor.resetTimeout(setPressureDischarge);
    if (setPressureDischarge != 0) { dischargeMotor.setControlEnable(true); controlEnableDischarge = true; }
    Serial.printf("Set Pressures: Inlet = %.2f bar, Discharge = %.2f bar\n", setPressureInlet, setPressureDischarge);
  } else { processCommand(); }
}
const char *Help_Message[] = {
  "=== COMMAND LIST ===", "start                  - Enable pressure control for both motors",
  "stop                   - Disable pressure control for both motors", "start_inlet            - Enable pressure control for inlet motor",
  "stop_inlet             - Disable pressure control for inlet motor", "start_discharge        - Enable pressure control for discharge motor",
  "stop_discharge         - Disable pressure control for discharge motor", "reset                  - Initiate reset for both motors (to pressure 0 for inlet, min pressure for discharge)",
  "steps                  - Display current step count and pressure readings", "maxStepsInlet:<value>  - Set max step count for inlet motor (Caution: not fully integrated with class yet)",
  "maxStepsDischarge:<value> - Set max step count for discharge motor (Caution: not fully integrated with class yet)", "config                 - Show current configuration values",
  "stream                 - Start streaming pressure data", "stop_stream            - Stop streaming pressure data",
  "help                   - Show this help message", "======================", "=== INPUT FORMAT ===",
  "Inlet and Discharge Pressures: '<inlet_pressure>,<discharge_pressure>' (e.g., -0.5,1.0)",
};
void printHelpMessage() { for (size_t i = 0; i < sizeof(Help_Message) / sizeof(Help_Message[0]); i++) { Serial.println(Help_Message[i]); } }

void processCommand() {
  serialStr = String(receivedChars); serialStr.trim();
  if (serialStr == "stop") {
    Serial.println(" OK! Both Motors Disabled"); inletMotor.setControlEnable(false); controlEnableInlet = false;
    dischargeMotor.setControlEnable(false); controlEnableDischarge = false; reset = false; 
  } else if (serialStr == "start") {
    Serial.println(" OK! Both Motors Enabled");
    inletMotor.setControlEnable(true); controlEnableInlet = true; inletMotor.resetTimeout(setPressureInlet); 
    inletMotor.setNotifyFlag(false); notifyInlet = false;
    dischargeMotor.setControlEnable(true); controlEnableDischarge = true; dischargeMotor.resetTimeout(setPressureDischarge);
    dischargeMotor.setNotifyFlag(false); notifyDischarge = false; reset = false; 
  } else if (serialStr == "stop_inlet") {
    Serial.println(" OK! Inlet Motor Disabled"); inletMotor.setControlEnable(false); controlEnableInlet = false;
  } else if (serialStr == "start_inlet") {
    Serial.println(" OK! Inlet Motor Enabled"); inletMotor.setControlEnable(true); controlEnableInlet = true;
    inletMotor.resetTimeout(setPressureInlet); inletMotor.setNotifyFlag(false); notifyInlet = false;
  } else if (serialStr == "stop_discharge") {
    Serial.println(" OK! Discharge Motor Disabled"); dischargeMotor.setControlEnable(false); controlEnableDischarge = false;
  } else if (serialStr == "start_discharge") {
    Serial.println(" OK! Discharge Motor Enabled"); dischargeMotor.setControlEnable(true); controlEnableDischarge = true;
    dischargeMotor.resetTimeout(setPressureDischarge); dischargeMotor.setNotifyFlag(false); notifyDischarge = false;
  } else if (serialStr.startsWith("maxStepsInlet")) {
    long val = serialStr.substring(13).toInt(); maxStepsInlet = val; 
    Serial.print(" OK! Global maxStepsInlet set to "); Serial.println(maxStepsInlet);
    Serial.println(" Note: Restart or re-init motor object required for this to take full effect if not dynamically settable.");
  } else if (serialStr.startsWith("maxStepsDischarge")) {
    long val = serialStr.substring(17).toInt(); maxStepsDischarge = val; 
    Serial.print(" OK! Global maxStepsDischarge set to "); Serial.println(maxStepsDischarge);
    Serial.println(" Note: Restart or re-init motor object required for this to take full effect if not dynamically settable.");
  } else if (serialStr == "steps") {
    Serial.printf(" Inlet Steps: %ld (Current P: %.2f bar, Target P: %.2f bar)\n", inletMotor.getStepCount(), currentPressureInlet, setPressureInlet);
    Serial.printf(" Discharge Steps: %ld (Current P: %.2f bar, Target P: %.2f bar)\n", dischargeMotor.getStepCount(), currentPressureDischarge, setPressureDischarge);
    Serial.printf(" Inlet motor enabled: %s\tDischarge motor enabled: %s\n",
                  inletMotor.isControlEnabled() ? "true" : "false", dischargeMotor.isControlEnabled() ? "true" : "false");
    Serial.printf(" Global reset flag: %s\n", reset ? "true" : "false");
  } else if (serialStr == "reset") {
    Serial.println(" Initiating reset for both motors..."); reset = true; 
    Serial.println("Directly attempting Inlet motor reset sequence via managePressure...");
    inletMotor.managePressure(false, true); 
    Serial.println("Directly attempting Discharge motor reset sequence via managePressure...");
    dischargeMotor.managePressure(false, true); 
    Serial.println("Re-attempting reset via direct blocking calls:");
    bool inletResetDone = inletMotor.resetMotor(); 
    if (inletResetDone) { Serial.println("Inlet motor reset sequence reported success by direct call."); } 
    else { Serial.println("Inlet motor reset sequence reported failure or timeout by direct call."); }
    bool dischargeResetDone = dischargeMotor.resetMotor(); 
    if (dischargeResetDone) { Serial.println("Discharge motor reset sequence reported success by direct call."); } 
    else { Serial.println("Discharge motor reset sequence reported failure or timeout by direct call."); }
    if (inletResetDone && dischargeResetDone) { Serial.println("Both motors reported reset completion by direct calls."); reset = false; } 
    else { Serial.println("One or both motors failed to reset via direct calls. Global 'reset' flag remains true."); }
  } else if (serialStr == "config") {
    Serial.println("==== CONFIG (via Motor Objects where applicable) ====");
    Serial.printf("Max Steps - Inlet: %ld, Discharge: %ld\n", inletMotor.getMaxSteps(), dischargeMotor.getMaxSteps());
    Serial.printf("Inlet Target Pressure for Reset: %.2f\n", inletMotor._minPressureForReset);
    Serial.printf("Discharge Target Pressure for Reset: %.2f\n", dischargeMotor._minPressureForReset);
    Serial.println("================");
  } else if (serialStr == "help") { printHelpMessage(); } 
  else if (serialStr == "stream") { streamPressure = true; Serial.println("Pressure streaming started"); } 
  else if (serialStr == "stop_stream") { streamPressure = false; Serial.println("Pressure streaming stopped"); } 
  else { Serial.println("Unknown command. Type 'help' for available options."); }
}
// Orphaned functions, logic moved/integrated
// void controlInletPressure() { /* ... */ }
// void controlDischargePressure() { /* ... */ }
