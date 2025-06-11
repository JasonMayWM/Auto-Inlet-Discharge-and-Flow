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

// === Global State Variables ===
unsigned long ticktime = 0;  // General-purpose timer variable, potentially for measuring elapsed time. Currently not used in critical logic.
int top = -1;                // Variable related to a stack data structure, possibly for command history or state management. Currently not used.
bool reset = false;          // Global flag to indicate if the system or motors should be in a reset state.

// === ADC & Sensor Constants ===
// ADC resolution for the RP2040's internal ADC (12-bit, 0-4095).
const float MY_ADC_RESOLUTION = 4095.0;

// Inlet Pressure Sensor Configuration
const float INLET_SENSOR_SCALE = 0.99;     // Scale factor for the inlet pressure sensor.
const float INLET_SENSOR_RANGE = 2.0;      // Sensor measurement range (e.g., -1 to +1 bar relative, so 2 bar total span).
const float INLET_SENSOR_OFFSET = -1.00f;  // Offset value for the inlet pressure sensor.
// Conversion factor: (Range * Scale) / ADC_Resolution. Used to convert ADC reading to pressure (bar).
const float INLET_PRESSURE_CONV = (INLET_SENSOR_RANGE * INLET_SENSOR_SCALE / MY_ADC_RESOLUTION);

// Discharge Pressure Sensor Configuration
const float DISCHARGE_SENSOR_SCALE = 1.0;     // Scale factor for the discharge pressure sensor.
const float DISCHARGE_SENSOR_RANGE = 10.0;    // Sensor measurement range (e.g., 0 to 10 bar).
const float DISCHARGE_SENSOR_OFFSET = -0.07;  // Offset value for the discharge pressure sensor.
// Conversion factor: (Range * Scale) / ADC_Resolution. Used to convert ADC reading to pressure (bar).
const float DISCHARGE_PRESSURE_CONV = (DISCHARGE_SENSOR_RANGE * DISCHARGE_SENSOR_SCALE / MY_ADC_RESOLUTION);

// Flow Sensor Configuration (Placeholders - Adjust to actual sensor)
const float FLOW_SENSOR_SCALE = 1.0;    // Scale factor for the flow sensor.
const float FLOW_SENSOR_RANGE = 3.0;    // Sensor measurement range (e.g., 0-5 L/min).
const float FLOW_SENSOR_OFFSET = 0.0f;  // Offset value for the flow sensor.
// Conversion factor: (Range * Scale) / ADC_Resolution. Used to convert ADC reading to flow units (e.g., L/min).
const float FLOW_CONV = (FLOW_SENSOR_RANGE * FLOW_SENSOR_SCALE / MY_ADC_RESOLUTION);
enum SensorType {INLET, DISCHARGE, FLOW}; //Sensoe types for the printBuffer() function.
// === Pin Definitions ===
// ADC Pins (Physical ADC channels on RP2040)
const int inletAnalogPin = 26;      // ADC0, corresponds to GPIO26. For reading inlet pressure.
const int dischargeAnalogPin = 27;  // ADC1, corresponds to GPIO27. For reading discharge pressure.
const int flowAnalogPin = 28;       // ADC2, corresponds to GPIO28. For reading flow meter.

// Inlet Motor Control Pins (GPIO pin numbers)
const int stepPinInlet = 4;  // STEP pin for the inlet motor driver.
const int dirPinInlet = 3;   // DIRECTION pin for the inlet motor driver.
const int enaPinInlet = 2;   // ENABLE pin for the inlet motor driver. (LOW to enable, HIGH to disable)

// Discharge Motor Control Pins (GPIO pin numbers)
const int stepPinDischarge = 8;  // STEP pin for the discharge motor driver.
const int dirPinDischarge = 7;   // DIRECTION pin for the discharge motor driver.
const int enaPinDischarge = 6;   // ENABLE pin for the discharge motor driver. (LOW to enable, HIGH to disable)

// Maximum steps for each motor (used in MotorController constructor).
// These define the operational range of the motors in terms of steps.
long maxStepsInlet = 325000;      // Maximum steps the inlet motor can take from its zero position.
long maxStepsDischarge = 350000;  // Maximum steps the discharge motor can take from its zero position.

// Step intervals (microseconds) for motor speed control (used in MotorController constructor).
long intervalRunInlet = 200;           // Target interval (µs) for inlet motor when running at speed (shorter = faster).
long initialIntervalInlet = 1000;      // Initial interval (µs) for inlet motor at start of acceleration (longer = slower).
long intervalRunDischarge = 200;       // Target interval (µs) for discharge motor at speed.
long initialIntervalDischarge = 1000;  // Initial interval (µs) for discharge motor at start of acceleration.

// Acceleration parameters (number of steps over which to accelerate/decelerate) (used in MotorController constructor).
long accelerationStepsInlet = 2000;      // Number of steps for inlet motor to reach full speed or stop.
long accelerationStepsDischarge = 1500;  // Number of steps for discharge motor to reach full speed or stop.

// Timeout duration for pressure control operations (milliseconds).
// If a motor tries to reach a set pressure and takes longer than this, it will time out.
const unsigned long timeoutDuration = 180000;  // 180 seconds.

// Pressure Variables
volatile float currentPressureInlet = 0.0;      // Current measured pressure for the inlet (bar). 'volatile' as it's updated by core1 and read by core0.
volatile float currentPressureDischarge = 0.0;  // Current measured pressure for the discharge (bar). 'volatile' for the same reason.
float setPressureInlet = 0.0;                   // Target pressure for the inlet (bar), set via serial command.
float setPressureDischarge = 0.0;               // Target pressure for the discharge (bar), set via serial command.

// Tolerances and Limits for Pressure Control (some passed to MotorController)
float pressureToleranceInlet = 0.025f;      // Allowed deviation (bar) from setPressureInlet to be considered "on target".
float pressureToleranceDischarge = 0.025f;  // Allowed deviation (bar) from setPressureDischarge.
float maxPressureInlet = 0.0f;             // Maximum allowable pressure for inlet (currently 0.0, meaning inlet is for vacuum/negative pressure).
float minimumPressureInlet = -1.0f;        // Minimum allowable pressure for inlet (e.g., -1.0 bar).
float maxPressureDischarge = 7.0f;         // Maximum allowable pressure for discharge (e.g., 7.0 bar).
float minimumPressureDischarge = 0.2f;     // Minimum pressure for discharge when resetting (acts as a reset threshold).

// Global Flags for System State
bool targetPressure = false;                     // True if inlet motor has reached its target pressure.
bool targetDischargePressure = false;            // True if discharge motor has reached its target pressure.
bool targetAchievedMsgPrintedInlet = false;      // Flag to control notification messages for inlet pressure achievement. True = message sent.
bool targetAchievedMsgPrintedDischarge = false;  // Flag to control notification messages for discharge pressure achievement. True = message sent.
bool controlEnableInlet = true;                  // Master enable/disable for inlet motor control logic.
bool controlEnableDischarge = true;              // Master enable/disable for discharge motor control logic.

// Global Overshoot Control Parameters (passed to MotorController constructor)
// These help manage and correct if the motor moves the pressure beyond the setpoint.
int reverseStepsInlet = 3000;      // Number of steps to reverse inlet motor if overshoot is detected.
float slowFactorInlet = 0.5f;      // Factor to slow down inlet motor during overshoot recovery (e.g., 0.5 = half speed).
int reverseStepsDischarge = 1000;  // Number of steps to reverse discharge motor if overshoot is detected.
float slowFactorDischarge = 0.5f;  // Factor to slow down discharge motor during overshoot recovery.

// === Circular Buffer Configuration ===
// Used for averaging sensor readings to smooth out noise.
const int arraySize = 250;              // Number of samples to store in the circular buffer.
int avgSampTime = 1000000 / arraySize;  // Target sampling time (µs) per sample to fill the buffer over 1 second.

// === Serial Communication ===
const byte numChars = 16;                   // Maximum number of characters expected in a serial command.
char receivedChars[numChars];               // Array to store incoming serial characters.
boolean newData = false;                    // Flag that becomes true when a complete new serial command is received (newline terminated).
String serialStr;                           // String object to process received serial commands.
bool streamPressure = false;                // Flag to enable/disable continuous streaming of pressure data via serial.
unsigned long lastStreamTime = 0;           // Timestamp of the last pressure data stream.
const unsigned long streamInterval = 1000;  // Interval (ms) for streaming pressure data. (1 second)

// Structure definition for a circular buffer.
// Used to store recent sensor readings and calculate their average.
struct CircularBuffer {
  int buffer[arraySize];  // Array to store buffer elements (scaled by 100, as integers).
  int start = 0;          // Index of the oldest element in the buffer.
  int count = 0;          // Number of elements currently in the buffer.

  // Adds a new value to the buffer. If the buffer is full, the oldest value is overwritten.
  void push(int value) {
    int idx = (start + count) % arraySize;  // Calculate index for the new value.
    buffer[idx] = value;
    if (count < arraySize) {
      count++;  // Increment count if buffer is not yet full.
    } else {
      start = (start + 1) % arraySize;  // Otherwise, overwrite oldest, so advance start index.
    }
  }

  // Calculates the average of the values currently in the buffer.
  float average() {
    if (count == 0) return 0;  // Avoid division by zero if buffer is empty.
    float sum = 0;
    for (int i = 0; i < count; i++) {
      sum += buffer[(start + i) % arraySize];
    }
    return sum / count;
  }
};

// Instances of CircularBuffer for sensor data smoothing.
CircularBuffer dischargeFilo;      // Buffer for discharge pressure readings.
CircularBuffer inletFilo;          // Buffer for inlet pressure readings.
CircularBuffer flowFilo;           // Buffer for simulated flow readings.
volatile float currentFlow = 0.0;  // Smoothed flow rate, calculated from averages. 'volatile' as it's updated by core1.

// Class to manage individual stepper motor control.
class MotorController {
public:
  // --- Pin Configuration ---
  int _stepPin;  // GPIO pin for STEP signal.
  int _dirPin;   // GPIO pin for DIRECTION signal.
  int _enaPin;   // GPIO pin for ENABLE signal.

  // --- Motor & Movement Parameters ---
  long _maxSteps;           // Maximum steps motor can travel from zero.
  long _initialInterval;    // Starting step interval (µs) for acceleration.
  long _runInterval;        // Target step interval (µs) at desired speed.
  long _accelerationSteps;  // Number of steps for acceleration/deceleration ramp.

  // Defines motor's physical direction relative to logical direction (true/false for stepMotor).
  // If true, `setDirection(true)` might mean DIR pin HIGH, `setDirection(false)` means DIR pin LOW.
  // If false, `setDirection(true)` might mean DIR pin LOW, `setDirection(false)` means DIR pin HIGH.
  // This adapts to how the motor is wired or its default rotation.
  // Step count always increases for 'forward' (true) and decreases for 'backward' (false).
  bool _motorDirectionSetting;

  // --- Pressure Control References & State ---
  volatile float& _currentPressureRef;     // Reference to global variable holding current pressure for this motor.
  float& _setPressureRef;                  // Reference to global variable holding target pressure for this motor.
  bool& _controlEnableRef;                 // Reference to global flag enabling/disabling this motor's control.
  bool& _targetAchievedMsgPrintedFlagRef;  // Reference to global flag for target achieved notification.
  float _minPressureForReset;              // Pressure threshold that indicates motor is reset (e.g., at zero steps).
  String _name;                            // Name of the motor (e.g., "Inlet", "Discharge") for logging.
  bool _adjustingMsgPrinted;               // Flag to track if "adjusting" message has been printed for the current operation.

  // --- Internal State Variables ---
  long _stepCount;                   // Current position of the motor in steps from zero.
  long _stepCountAcc;                // Accumulator for steps taken during current acceleration/deceleration phase.
  bool _previousDirection;           // Last logical direction motor moved (true=forward, false=backward).
  unsigned long _motorLastStepTime;  // Timestamp (µs) of the last step taken.
  long _currentInterval;             // Current step interval (µs), dynamically adjusted for acc/dec.

  // --- Speed Slowdown Parameters (approaching target) ---
  float _slowdownPressureThreshold1;  // Pressure difference threshold to start first slowdown.
  float _slowdownFactor1;             // Factor to multiply _runInterval by for first slowdown.
  float _slowdownPressureThreshold2;  // Pressure difference threshold to start second (more significant) slowdown.
  float _slowdownFactor2;             // Factor for second slowdown. (-1.0f for threshold2 disables it)

  // --- Timeout Control ---
  unsigned long _pressureControlStartTime;  // Timestamp (ms) when current pressure control operation started.
  bool _timedOut;                           // Flag: true if pressure control has timed out.

  // --- Overshoot Control Members ---
  int _overshootReverseSteps;   // Number of steps to reverse if an overshoot is detected.
  float _overshootSlowFactor;   // Speed reduction factor during overshoot recovery.
  bool _isRecoveringOvershoot;  // Flag: true if currently recovering from an overshoot.
  float _pressureTolerance;     // Pressure tolerance for detecting overshoot and confirming recovery.

  // Constructor for MotorController
  MotorController(int stepPin, int dirPin, int enaPin,
                  long maxSteps, long initialInterval, long runInterval, long accelerationSteps,
                  bool motorDirectionSetting,
                  volatile float& currentPressureRef, float& setPressureRef,
                  bool& controlEnableRef, bool& targetAchievedMsgPrintedFlagRef,
                  float minPressureForReset,
                  float slowdownPressureThreshold1, float slowdownFactor1,
                  float slowdownPressureThreshold2, float slowdownFactor2,
                  String name,
                  int overshootReverseSteps, float overshootSlowFactor, float pressureToleranceVal)
    : _stepPin(stepPin), _dirPin(dirPin), _enaPin(enaPin),
      _maxSteps(maxSteps), _initialInterval(initialInterval), _runInterval(runInterval), _accelerationSteps(accelerationSteps),
      _motorDirectionSetting(motorDirectionSetting),
      _currentPressureRef(currentPressureRef), _setPressureRef(setPressureRef),
      _controlEnableRef(controlEnableRef), _targetAchievedMsgPrintedFlagRef(targetAchievedMsgPrintedFlagRef),
      _minPressureForReset(minPressureForReset),
      _slowdownPressureThreshold1(slowdownPressureThreshold1), _slowdownFactor1(slowdownFactor1),
      _slowdownPressureThreshold2(slowdownPressureThreshold2), _slowdownFactor2(slowdownFactor2),
      _name(name),
      _overshootReverseSteps(overshootReverseSteps), _overshootSlowFactor(overshootSlowFactor),
      _pressureTolerance(pressureToleranceVal), _isRecoveringOvershoot(false)  // Initialize members
  {
    // Initialize internal state
    _stepCount = 0;                       // Start at zero steps.
    _stepCountAcc = 0;                    // Reset acceleration step count.
    _previousDirection = true;            // Default previous direction.
    _motorLastStepTime = 0;               // Initialize last step time.
    _currentInterval = _initialInterval;  // Start with initial (slower) interval.
    _pressureControlStartTime = 0;        // No timeout active initially.
    _timedOut = false;                    // Not timed out.
    _adjustingMsgPrinted = false;         // Initialize new flag

    // Configure GPIO pins for motor control
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_enaPin, OUTPUT);
    digitalWrite(_stepPin, LOW);  // Ensure step pin is initially low.
    digitalWrite(_enaPin, HIGH);  // Disable motor driver initially (HIGH = disabled for TB6600).
  }

  // Sets the motor direction pin based on the logical direction and _motorDirectionSetting.
  // Also resets acceleration step count if direction changes.
  void setDirection(bool direction) {  // true for forward/increasing steps, false for backward/decreasing steps
    if (_motorDirectionSetting) {
      digitalWrite(_dirPin, !direction);  // Inverted if _motorDirectionSetting is true
    } else {
      digitalWrite(_dirPin, direction);  // Normal if _motorDirectionSetting is false
    }
    // If logical direction changes, reset acceleration ramp.
    if (direction != _previousDirection) {
      _stepCountAcc = 0;
      _previousDirection = direction;
    }
  }

  // Enables or disables the motor driver.
  void enableMotor(bool enable) {
    digitalWrite(_enaPin, enable ? LOW : HIGH);  // LOW enables TB6600, HIGH disables.
  }

  // Returns true if motor control is currently enabled.
  bool isControlEnabled() {
    return _controlEnableRef;
  }

  // Sets the motor control enable flag.
  void setControlEnable(bool enable) {
    _controlEnableRef = enable;
  }

  // Returns the current step count of the motor.
  long getStepCount() {
    return _stepCount;
  }

  // Returns the current step count of the motor + 2500 step to allow the motor to go further backwards.
  long increaseStepCount() {
    _stepCount = _stepCount + 2500;
    return _stepCount;
  }

  // Returns the maximum allowed steps for the motor.
  long getMaxSteps() {
    return _maxSteps;
  }

  // Sets the notification flag (e.g., when target pressure is achieved).
  void setTargetAchievedMsgPrinted(bool val) {
    _targetAchievedMsgPrintedFlagRef = val;
  }

  // Gets the current state of the notification flag.
  bool getTargetAchievedMsgPrinted() const {
    return _targetAchievedMsgPrintedFlagRef;
  }

  // Returns true if the motor is currently in overshoot recovery mode.
  bool isRecoveringOvershoot() const {
    return _isRecoveringOvershoot;
  }

  // Calculates the appropriate step interval for acceleration, deceleration, and slowdowns.
  void calculateInterval() {
    // If recovering from overshoot, use a fixed slow interval.
    if (_isRecoveringOvershoot) {
      _currentInterval = (unsigned long)(_runInterval / _overshootSlowFactor);
      if (_currentInterval == 0) _currentInterval = 1;  // Prevent division by zero or too fast.
      return;                                           // Skip normal acceleration/deceleration logic.
    }

    float error = fabs(_currentPressureRef - _setPressureRef);  // Absolute difference to target.

    // Acceleration phase: Linearly decrease interval from _initialInterval to _runInterval.
    if (_stepCountAcc < _accelerationSteps) {
      float progress = (float)_stepCountAcc / _accelerationSteps;
      _currentInterval = _initialInterval - (progress * (_initialInterval - _runInterval));
      _stepCountAcc++;
    } else {
      // Constant speed phase (or deceleration handled by slowdown).
      _currentInterval = _runInterval;
    }

    // Slowdown phase: If close to setpoint, increase interval (slow down).
    if (_setPressureRef != 0) {  // Only apply slowdown if there's a target.
      // Check second (more aggressive) slowdown threshold first.
      if (_slowdownPressureThreshold2 != -1.0f && error < _slowdownPressureThreshold2) {
        _currentInterval = _runInterval * _slowdownFactor2;  // Larger interval = slower.
      }
      // Check first slowdown threshold.
      else if (error < _slowdownPressureThreshold1) {
        _currentInterval = _runInterval * _slowdownFactor1;
      }
    }
    // Ensure interval is not zero or negative.
    if (_currentInterval <= 0) { _currentInterval = _runInterval; }
  }

  // Executes a single motor step if conditions are met.
  // `direction`: true for forward (increment step count), false for backward (decrement).
  void stepMotor(bool direction) {
    if (!_controlEnableRef) return;  // Do nothing if motor control is disabled.

    // Check if enough time has passed since the last step, based on current interval.
    if ((int32_t)micros() - _motorLastStepTime < _currentInterval) { return; }

    setDirection(direction);  // Set motor direction.
    calculateInterval();      // Update _currentInterval for next step (accel/decel).

    // Perform step: pulse STEP pin.
    digitalWrite(_stepPin, HIGH);
    delayMicroseconds(5);  // Brief delay to ensure driver registers the pulse.
    digitalWrite(_stepPin, LOW);

    _motorLastStepTime = micros();     // Record time of this step.
    _stepCount += direction ? 1 : -1;  // Update step count based on direction.

    // Optional: Notify that motor is adjusting (can be verbose).
    // This specific message should only appear if the target was previously achieved and now we are moving again.
    if (getTargetAchievedMsgPrinted() && !_adjustingMsgPrinted && _controlEnableRef && _setPressureRef != 0) {  // Condition changed here
      Serial.print(_name);
      Serial.println(" adjusting...");
      _adjustingMsgPrinted = true;
      setTargetAchievedMsgPrinted(false);
    }
  }

  // Resets the motor to its zero position or a defined reset pressure.
  // Returns true if reset was successful, false on timeout.
  bool resetMotor() {
    enableMotor(true);       // Ensure motor is enabled for movement.
    setControlEnable(true);  // Ensure control logic is active.
    Serial.print(_name);
    Serial.println(" motor resetting...");
    unsigned long resetOpStartTime = millis();          // Timeout for reset operation.
    const unsigned long resetOperationTimeout = 30000;  // 30 seconds for reset.

    while (millis() - resetOpStartTime < resetOperationTimeout) {
      // Condition for successful reset (e.g., pressure at or below a minimum).
      bool conditionMet = (_currentPressureRef <= _minPressureForReset);
      if (conditionMet) {
        _stepCount = 0;
        _stepCountAcc = 0;
        _setPressureRef = 0;  // Reset state variables.
        Serial.print(_name);
        Serial.println(" reset complete (pressure condition met).");
        return true;
      }

      // Step motor backwards slowly until conditionMet or timeout.
      // Uses a fixed slow interval for reset.
      if ((int32_t)micros() - _motorLastStepTime >= (_initialInterval > 500 ? 500 : _initialInterval)) {
        setDirection(false);                                                 // Move backward.
        _currentInterval = _initialInterval > 500 ? 500 : _initialInterval;  // Fixed slow speed.
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(_stepPin, LOW);
        _motorLastStepTime = micros();
        _stepCount += -1;  // Decrement step count.
      }
    }
    Serial.print(_name);
    Serial.println(" motor reset timeout!");
    enableMotor(false);  // Disable motor on timeout.
    return false;
  }

  // Returns the timestamp (µs) of the last motor step.
  unsigned long getLastStepTime() {
    return _motorLastStepTime;
  }

  // Returns the current step interval (µs).
  long getCurrentInterval() {
    return _currentInterval;
  }

  // Resets the pressure control timeout timer.
  // Called when a new set pressure is received.
  void resetTimeout(float newSetPressure) {
    if (newSetPressure != 0) {  // If there's a non-zero target, start timeout timer.
      _pressureControlStartTime = millis();
      _timedOut = false;
    } else {  // If target is zero, effectively no timeout needed for "holding" at zero.
      _pressureControlStartTime = 0;
      _timedOut = false;
    }
  }

  // Main pressure management function for the motor.
  // `targetPressureFlag`: true if the desired pressure has been reached and should be maintained.
  // `globalResetFlag`: true if a global system reset is requested.
  void managePressure(bool targetPressureFlag, bool globalResetFlag) {
    // If motor has timed out, keep it disabled and do nothing.
    if (_timedOut) {
      enableMotor(false);
      return;
    }

    // Handle global reset command.
    if (globalResetFlag) {
      if (resetMotor()) { /* Individual motor reset successful. */
      }
      // `reset` flag in main loop will be cleared if all motors reset successfully.
      return;
    }

    // Enable motor if not yet at target pressure, disable if at target.
    // This allows the motor to hold position by being disabled once target is met.
    enableMotor(!targetPressureFlag);

    // Check for timeout if motor is active and trying to reach a non-zero setpoint.
    if (!targetPressureFlag && _setPressureRef != 0 && isControlEnabled()) {
      if (_pressureControlStartTime != 0 && millis() - _pressureControlStartTime > timeoutDuration) {
        Serial.print(_name);
        Serial.println(" motor pressure control timeout!");
        _timedOut = true;         // Set timeout flag.
        setControlEnable(false);  // Disable control.
        enableMotor(false);       // Disable motor hardware.
        return;
      }
    }

    // If motor control is disabled or target pressure is already achieved, do nothing further.
    if (!isControlEnabled() || targetPressureFlag) { return; }

    // --- Overshoot Detection and Recovery Logic ---
    if (!_isRecoveringOvershoot) {  // If not already recovering...
      bool overshootDetected = false;
      // Detect overshoot: current pressure is beyond setpoint + tolerance.
      // Handles both positive and negative setpoints.
      if ((_setPressureRef > 0 && _currentPressureRef > _setPressureRef + _pressureTolerance) || (_setPressureRef < 0 && _currentPressureRef < _setPressureRef - _pressureTolerance)) {
        overshootDetected = true;
      }

      if (overshootDetected) {
        Serial.print(_name);
        Serial.println(" overshoot detected!");
        _isRecoveringOvershoot = true;  // Enter recovery mode.

        // Determine the physical direction pin state for reverse.
        // This depends on _motorDirectionSetting and the last logical direction (_previousDirection).
        bool reverseDirForPin = _previousDirection ? !_motorDirectionSetting : _motorDirectionSetting;
        digitalWrite(_dirPin, reverseDirForPin);  // Set DIR pin for reverse.

        // Execute a fixed number of reverse steps at a fixed (fast) speed.
        for (int i = 0; i < _overshootReverseSteps; i++) {
          if (!isControlEnabled()) break;  // Stop if control is disabled during this.
          digitalWrite(_stepPin, HIGH);
          delayMicroseconds(200);  // Quick steps.
          digitalWrite(_stepPin, LOW);
          delayMicroseconds(200);
          // Note: _stepCount is not updated here to simplify recovery logic.
          // The goal is to quickly move back, then let normal control resume slowly.
        }
        _motorLastStepTime = micros();  // Update last step time to allow recovery step soon.
        return;                         // Exit managePressure for this cycle; recovery will proceed in next call.
      }
    } else {  // _isRecoveringOvershoot is true
      // This block handles motor movement when recovering from an overshoot.
      // The goal is to bring the pressure back to the setpoint slowly.

      bool directionToSetPoint = (_currentPressureRef < _setPressureRef);
      // directionToSetPoint will be 'true' if current pressure is less than set pressure (needs positive steps/pressure increase)
      // directionToSetPoint will be 'false' if current pressure is greater than set pressure (needs negative steps/pressure decrease)

      // Check if we need to step the motor
      bool shouldStep = true;
      if (_setPressureRef == 0 && !directionToSetPoint && _stepCount <= 0) {
        // Specific condition to prevent negative steps when target is 0 bar:
        // - Target pressure (_setPressureRef) is 0.
        // - Motor wants to move in 'false' direction (directionToSetPoint is false), which means current pressure is > 0 and we want to reduce it by reducing steps.
        // - Step count (_stepCount) is already at or below 0.
        // In this case, we should not step further in the 'false' direction.
        shouldStep = false;
        // It's possible that even if steps are 0, pressure isn't exactly 0.
        // The recovery completion check below will handle exiting the recovery state.
      }

      if (shouldStep) {
        // calculateInterval (called by stepMotor) will use the slow factor because _isRecoveringOvershoot is true
        stepMotor(directionToSetPoint);
      }

      // Check if overshoot recovery is complete
      if (fabs(_currentPressureRef - _setPressureRef) <= (_pressureTolerance / 2.0f)) {
        // Recovery is considered complete if current pressure is very close to the setpoint.
        Serial.print(_name);
        Serial.println(" overshoot recovery complete.");
        _isRecoveringOvershoot = false;
        // If _setPressureRef is 0 and we just completed recovery,
        // and if shouldStep was false (because _stepCount was <=0),
        // then _stepCount might be 0 or negative.
        // The main logic after this recovery block will then use:
        // "else if (_setPressureRef == 0 && _stepCount > 0)" which won't run if _stepCount <=0.
        // If _stepCount became negative due to past logic, this fix aims to prevent it from getting more negative here.
        // A remaining negative value would be a pre-existing condition or from how pressure maps to steps.
        // For now, the goal is to stop it from decrementing further if already at/below zero when target is zero.
      }
      return;  // Return after a recovery step attempt
    }

    // --- Normal Stepping Logic (if no overshoot detected and not recovering) ---
    // This logic decides whether to step forward, backward, or do nothing based on current vs. set pressure.

    // Condition to step backward (decrease pressure / decrease steps):
    // - Setpoint is non-zero.
    // - Motor is not at zero steps already.
    // - Current pressure is above setpoint (for positive setpoints) OR below setpoint (for negative setpoints).
    if (_setPressureRef != 0 && _stepCount > 0 && ((_currentPressureRef > _setPressureRef && _setPressureRef > 0) || (_currentPressureRef < _setPressureRef && _setPressureRef < 0))) {
      stepMotor(false);  // Step backward.
    }
    // Condition to step forward (increase pressure / increase steps):
    // - Setpoint is non-zero.
    // - Motor is not at max steps already.
    // - Current pressure is below setpoint (for positive setpoints) OR above setpoint (for negative setpoints).
    else if (_setPressureRef != 0 && _stepCount < _maxSteps && ((_currentPressureRef < _setPressureRef && _setPressureRef > 0) || (_currentPressureRef > _setPressureRef && _setPressureRef < 0))) {
      stepMotor(true);  // Step forward.
    }
    // Condition: At maximum steps but not at target pressure.
    else if (_stepCount >= _maxSteps && _setPressureRef != 0) {
      setControlEnable(false);  // Disable motor to prevent further movement.
      Serial.print(_name);
      Serial.println(" motor at maximum step count. Motor disabled, reset pressure to '0' and then 'start' to enable.");
    }
    // Condition: Setpoint is zero, and motor is not yet at zero steps.
    else if (_setPressureRef == 0 && _stepCount > 0) {
      stepMotor(false);  // Step backward towards zero.
    }
    // If none of the above, motor holds position (or is handled by targetPressureFlag logic at the start).
  }
};

// === MotorController Object Instantiation ===
MotorController inletMotor(
  stepPinInlet, dirPinInlet, enaPinInlet,                                                     // Pin definitions
  maxStepsInlet, initialIntervalInlet, intervalRunInlet, accelerationStepsInlet,              // Movement parameters
  true,                                                                                       // _motorDirectionSetting: true = DIR pin is inverted relative to logical 'forward'
  currentPressureInlet, setPressureInlet, controlEnableInlet, targetAchievedMsgPrintedInlet,  // References to global vars
  maxPressureInlet,                                                                           // _minPressureForReset (using maxPressureInlet for inlet, as it's usually 0 or negative)
  0.2f, 16.0f, 0.15f, 40.0f,                                                                  // Slowdown thresholds and factors
  "Inlet",                                                                                    // Name
  reverseStepsInlet, slowFactorInlet, pressureToleranceInlet                                  // Overshoot parameters
);

MotorController dischargeMotor(
  stepPinDischarge, dirPinDischarge, enaPinDischarge,                                                         // Pin definitions
  maxStepsDischarge, initialIntervalDischarge, intervalRunDischarge, accelerationStepsDischarge,              // Movement parameters
  true,                                                                                                       // _motorDirectionSetting
  currentPressureDischarge, setPressureDischarge, controlEnableDischarge, targetAchievedMsgPrintedDischarge,  // References
  minimumPressureDischarge,                                                                                   // _minPressureForReset (discharge resets to a minimum positive pressure)
  0.2f, 8.0f, 0.0f, 1.0f,                                                                                     // Slowdown: Threshold2 is 0.0f, Factor2 is 1.0f (effectively one slowdown stage)
  "Discharge",                                                                                                // Name
  reverseStepsDischarge, slowFactorDischarge, pressureToleranceDischarge                                      // Overshoot parameters
);

// Forward declaration for the function running on core1.
void pressureSensorCore1();

// --- Arduino Setup Function ---
// Initializes serial communication, launches core1, and prints welcome messages.
// Core0 will run setup() and then loop().
void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud.

  adc_init();                         // Initialize the ADC peripheral
  adc_gpio_init(inletAnalogPin);      // Initialize GPIO26 for ADC input (ADC0)
  adc_gpio_init(dischargeAnalogPin);  // Initialize GPIO27 for ADC input (ADC1)
  adc_gpio_init(flowAnalogPin);       // Initialize GPIO28 for ADC input (ADC2)

  // Launch the pressureSensorCore1 function on the second core (core1).
  // Core1 will independently run pressureSensorCore1 in a loop.
  multicore_launch_core1(pressureSensorCore1);

  while (!Serial)
    ;  // Wait for Serial port to be ready (optional, good for some boards).

  Serial.println("Inlet and Discharge Pressure Control Software Version 0.05 (MotorController Refactor)");
  Serial.println("Enter 'help' to list available serial commands");
}

// --- Arduino Main Loop Function (Core0) ---
// Handles serial input, manages motor target achievement logic, and calls motor management functions.
void loop() {
  serial();  // Check for and process incoming serial commands.

  // --- Inlet Motor Target Logic ---
  bool inletTargetAchieved = false;  // Local flag for current loop iteration.
  if (!reset) {                      // Only check for target if system is not in reset mode.
    // Case 1: Negative set pressure (typical for vacuum/inlet).
    if (setPressureInlet < 0) {
      // Target achieved if current pressure is within tolerance AND not currently recovering from an overshoot.
      if (fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet && !inletMotor.isRecoveringOvershoot()) {
        inletTargetAchieved = true;
        if (!inletMotor.getTargetAchievedMsgPrinted()) {  // If notification not yet sent...
          Serial.printf("Inlet target pressure %.2f bar achieved\n", currentPressureInlet);
          inletMotor.setTargetAchievedMsgPrinted(true);  // Mark notification as sent.
          inletMotor._adjustingMsgPrinted = false;
        }
      }
    }
    // Case 2: Zero set pressure (return to zero/home).
    else if (setPressureInlet == 0) {
      // Target achieved if pressure is near zero AND step count is zero.
      // Overshoot recovery state is less critical here, focus is on reaching physical zero.
      if (inletMotor.getStepCount() == 0) {
        inletTargetAchieved = true;
        if (!inletMotor.getTargetAchievedMsgPrinted()) {
          Serial.println("Inlet zero position achieved");
          inletMotor.setTargetAchievedMsgPrinted(true);
          inletMotor._adjustingMsgPrinted = false;
        }
      }
    }
    // Case 3: Positive set pressure (less common for inlet, but handled).
    else {  // This implies setPressureInlet > 0
      if (fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet && !inletMotor.isRecoveringOvershoot()) {
        inletTargetAchieved = true;
        if (!inletMotor.getTargetAchievedMsgPrinted()) {
          Serial.printf("Inlet target pressure %.2f bar achieved\n", currentPressureInlet);
          inletMotor.setTargetAchievedMsgPrinted(true);
          inletMotor._adjustingMsgPrinted = false;
        }
      }
    }
  }

  targetPressure = inletTargetAchieved;  // Update global flag used by inletMotor.managePressure().

  // The _targetAchievedMsgPrintedFlagRef (and corresponding global targetAchievedMsgPrintedInlet)
  // should now only be reset by a new command (in processSerialInput or processCommand).
  // It is no longer reset here if pressure drifts out of tolerance.
  // The 'adjusting...' message logic in stepMotor relies on this flag remaining true
  // until a new command or explicit state change.
  // End of new logic for inlet targetPressure determination

  // Call the inlet motor's pressure management function.
  // Passes whether target is achieved and current global reset state.
  inletMotor.managePressure(targetPressure, reset);

  // --- Discharge Motor Target Logic --- (Similar structure to inlet)
  if (!reset && fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge && setPressureDischarge != 0) {
    targetDischargePressure = true;
    if (!dischargeMotor.getTargetAchievedMsgPrinted()) {
      Serial.printf("Discharge target pressure %.2f bar achieved\n", currentPressureDischarge);
      dischargeMotor.setTargetAchievedMsgPrinted(true);
      dischargeMotor._adjustingMsgPrinted = false;
    }
  } else if (setPressureDischarge == 0 && dischargeMotor.getStepCount() == 0 && !reset) {
    targetDischargePressure = true;
    if (!dischargeMotor.getTargetAchievedMsgPrinted()) {
      Serial.println("Discharge zero position achieved");
      dischargeMotor.setTargetAchievedMsgPrinted(true);
      dischargeMotor._adjustingMsgPrinted = false;
    }
  } else {
    targetDischargePressure = false;
    // The _targetAchievedMsgPrintedFlagRef (and corresponding global targetAchievedMsgPrintedDischarge)
    // should now only be reset by a new command.
  }
  // Call the discharge motor's pressure management function.
  dischargeMotor.managePressure(targetDischargePressure, reset);

  // --- Pressure Data Streaming ---
  // If streaming is enabled and interval has passed, print pressure data.
  if (streamPressure && millis() - lastStreamTime >= streamInterval) {
    lastStreamTime = millis();  // Reset stream timer.
    // Print data from circular buffers (which are updated by core1).
    printBuffer(inletFilo, "Inlet", INLET);
    printBuffer(dischargeFilo, "Discharge", DISCHARGE);
    printBuffer(flowFilo, "Flow", FLOW);
    delay(10);  // Small delay to allow serial buffer to flush.
  }
}

// --- Pressure Sensor Reading Core (Core1) ---
// This function runs on the RP2040's second core.
// It reads ADC values for inlet pressure, discharge pressure, and flow.
// It converts these values to appropriate units and updates global variables.
// It also populates circular buffers for averaging.
void pressureSensorCore1() {
  while (true) {  // Continuous loop on core1.
    // Read Inlet Pressure
    adc_select_input(0);
    sleep_us(10);
    uint16_t rawInlet = adc_read();
    inletFilo.push(rawInlet);
    currentPressureInlet = (inletFilo.average() * INLET_PRESSURE_CONV) + INLET_SENSOR_OFFSET;

    // Read Discharge Pressure
    adc_select_input(1);
    sleep_us(10);
    uint16_t rawDischarge = adc_read();
    dischargeFilo.push(rawDischarge);
    currentPressureDischarge = (dischargeFilo.average() * DISCHARGE_PRESSURE_CONV) + DISCHARGE_SENSOR_OFFSET;

    // Read Flow
    adc_select_input(2);
    sleep_us(10);
    uint16_t rawFlow = adc_read();
    flowFilo.push(rawFlow);
    // Use global constants for flow conversion
    currentFlow = (flowFilo.average() * FLOW_CONV) + FLOW_SENSOR_OFFSET;

    sleep_us(avgSampTime);  // Sleep for the average sample time to maintain ~desired update rate.
  }
}

// --- Print Buffer Function ---
// Prints the contents of a circular buffer to serial, formatted as CSV.
// `label`: Name for the data series (e.g., "Inlet").
// `isInlet`: (Currently unused in function logic, but passed) Could be for specific formatting.
void printBuffer(CircularBuffer& cb, const char* label, SensorType type) {
  Serial.printf("%s", label);  // Print data series label.

  for (int i = 0; i < cb.count; i++) {
    int idx = (cb.start + i) % arraySize;
    uint16_t rawValue = cb.buffer[idx];  // Get raw ADC value.

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

    Serial.printf(",%.2f", convertedValue);  // Print converted value.
  }

  Serial.println();  // Newline after all values.
  delay(100);        // Delay to avoid flooding serial output.
}

// --- Serial Handling Functions ---
// `serial()`: Main serial routine, called repeatedly from loop().
void serial() {
  recvWithEndMarker();  // Check for incoming serial data.
  showNewNumber();      // Process if new data is available.
}

// `recvWithEndMarker()`: Reads serial data until a newline character is received or buffer is full.
// Sets `newData` flag when a complete command is ready.
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

// `showNewNumber()`: If `newData` is true, processes the received command and resets the flag.
void showNewNumber() {
  if (newData) {
    processSerialInput();  // Main command processing logic.
    Serial.println();      // Print a blank line for cleaner serial output.
    newData = false;       // Reset flag.
  }
}

// `processSerialInput()`: Parses incoming serial commands.
// Differentiates between pressure setting commands (e.g., "-0.5,2.0") and text commands.
void processSerialInput() {
  // Check if the received string contains a comma, indicating a pressure set command.
  char* commaPosition = strchr(receivedChars, ',');
  if (commaPosition != nullptr) {                          // Comma found: interpret as "<inlet_pressure>,<discharge_pressure>"
    *commaPosition = '\0';                                 // Split string at comma by replacing comma with null terminator.
    float newInletPressure = atof(receivedChars);          // Convert first part to float (inlet pressure).
    float newDischargePressure = atof(commaPosition + 1);  // Convert second part (discharge pressure).

    // Apply safety limits to received pressures.
    // Inlet typically operates <= 0 bar. This clamps it to 0 if positive value is sent.
    if (newInletPressure < -1.0f) newInletPressure = 0.0f;      // Min limit (e.g. -1 bar, or 0 if trying to go more negative)
    else if (newInletPressure > 0.0f) newInletPressure = 0.0f;  // Max limit (0 bar for this config)

    // Discharge typically operates >= 0 bar.
    if (newDischargePressure < 0.0f) newDischargePressure = 0.0f;       // Min limit (0 bar)
    else if (newDischargePressure > 7.0f) newDischargePressure = 0.0f;  // Max limit (e.g. 7 bar, or 0 if trying to go higher)

    setPressureInlet = newInletPressure;
    setPressureDischarge = newDischargePressure;

    // Reset notification flags for both motors (global flags are updated by reference via the set method)
    inletMotor.setTargetAchievedMsgPrinted(false);
    inletMotor._adjustingMsgPrinted = false;

    dischargeMotor.setTargetAchievedMsgPrinted(false);
    dischargeMotor._adjustingMsgPrinted = false;

    // First confirmation print (this one always prints the target setpoints)
    Serial.printf("Set Pressures: Inlet = %.2f bar, Discharge = %.2f bar\n", setPressureInlet, setPressureDischarge);

    // == New Immediate Feedback Logic for Inlet Motor ==
    if (setPressureInlet == 0) {
      if (inletMotor.getStepCount() == 0 && fabs(currentPressureInlet) <= pressureToleranceInlet) {
        Serial.println("Inlet zero position achieved");
        inletMotor.setTargetAchievedMsgPrinted(true);  // Message printed, so set this true
      } else {
        Serial.println("Inlet adjusting...");
        inletMotor._adjustingMsgPrinted = true;  // "adjusting" message printed
      }
    } else {  // setPressureInlet != 0
      if (fabs(currentPressureInlet - setPressureInlet) <= pressureToleranceInlet && !inletMotor.isRecoveringOvershoot()) {
        Serial.printf("Inlet target pressure %.2f bar achieved\n", currentPressureInlet);
        inletMotor.setTargetAchievedMsgPrinted(true);  // Message printed, so set this true
      } else {
        Serial.println("Inlet adjusting...");
        inletMotor._adjustingMsgPrinted = true;  // "adjusting" message printed
      }
    }

    // == New Immediate Feedback Logic for Discharge Motor ==
    if (setPressureDischarge == 0) {
      if (dischargeMotor.getStepCount() == 0 && fabs(currentPressureDischarge) <= pressureToleranceDischarge) {
        Serial.println("Discharge zero position achieved");
        dischargeMotor.setTargetAchievedMsgPrinted(true);  // Message printed, so set this true
      } else {
        Serial.println("Discharge adjusting...");
        dischargeMotor._adjustingMsgPrinted = true;  // "adjusting" message printed
      }
    } else {  // setPressureDischarge != 0
      if (fabs(currentPressureDischarge - setPressureDischarge) <= pressureToleranceDischarge && !dischargeMotor.isRecoveringOvershoot()) {
        Serial.printf("Discharge target pressure %.2f bar achieved\n", currentPressureDischarge);
        dischargeMotor.setTargetAchievedMsgPrinted(true);  // Message printed, so set this true
      } else {
        Serial.println("Discharge adjusting...");
        dischargeMotor._adjustingMsgPrinted = true;  // "adjusting" message printed
      }
    }

    // Ensure motors are enabled to make adjustments or move to zero, and reset timeouts.
    inletMotor.setControlEnable(true);
    controlEnableInlet = true;
    inletMotor.resetTimeout(setPressureInlet);

    dischargeMotor.setControlEnable(true);
    controlEnableDischarge = true;
    dischargeMotor.resetTimeout(setPressureDischarge);


  } else {  // No comma: interpret as a text command.
    processCommand();
  }
}

// Help message content: array of strings.
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

// Prints the help message to serial.
void printHelpMessage() {
  for (size_t i = 0; i < sizeof(Help_Message) / sizeof(Help_Message[0]); i++) {
    Serial.println(Help_Message[i]);
  }
}

// `processCommand()`: Handles text-based serial commands.
void processCommand() {
  serialStr = String(receivedChars);  // Convert char array to Arduino String object.
  serialStr.trim();                   // Remove leading/trailing whitespace.

  if (serialStr == "stop") {
    Serial.println(" OK! Both Motors Disabled");
    inletMotor.setControlEnable(false);
    controlEnableInlet = false;
    dischargeMotor.setControlEnable(false);
    controlEnableDischarge = false;
    reset = false;  // Ensure reset flag is cleared if motors are stopped.
  } else if (serialStr == "start") {
    Serial.println(" OK! Both Motors Enabled");
    inletMotor.setControlEnable(true);
    controlEnableInlet = true;
    inletMotor.resetTimeout(setPressureInlet);  // Reset timeout on start.
    inletMotor.setTargetAchievedMsgPrinted(false);
    targetAchievedMsgPrintedInlet = false;  // Clear notification.
    inletMotor._adjustingMsgPrinted = false;

    dischargeMotor.setControlEnable(true);
    controlEnableDischarge = true;
    dischargeMotor.resetTimeout(setPressureDischarge);
    dischargeMotor.setTargetAchievedMsgPrinted(false);
    targetAchievedMsgPrintedDischarge = false;
    dischargeMotor._adjustingMsgPrinted = false;
    reset = false;  // Clear reset flag.
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
  } else if (serialStr == "back_inlet") {
    Serial.println(" OK! Move inlet motor backwards");
    inletMotor.increaseStepCount();

  } else if (serialStr == "back_discharge") {
    Serial.println(" OK! Move discharge motor backwards");
    dischargeMotor.increaseStepCount();

  } else if (serialStr.startsWith("maxStepsInlet")) {  // Command: "maxStepsInlet:VALUE"
    long val = serialStr.substring(13).toInt();        // Extract value after colon.
    maxStepsInlet = val;                               // Update global variable (Note: MotorController object itself is not updated here dynamically).
    Serial.print(" OK! Global maxStepsInlet set to ");
    Serial.println(maxStepsInlet);
    Serial.println(" Note: Restart or re-init motor object required for this to take full effect if not dynamically settable.");
  } else if (serialStr.startsWith("maxStepsDischarge")) {  // Command: "maxStepsDischarge:VALUE"
    long val = serialStr.substring(17).toInt();
    maxStepsDischarge = val;
    Serial.print(" OK! Global maxStepsDischarge set to ");
    Serial.println(maxStepsDischarge);
    Serial.println(" Note: Restart or re-init motor object required for this to take full effect if not dynamically settable.");
  } else if (serialStr == "steps") {
    Serial.printf(" Inlet Steps: %ld (Current P: %.2f bar, Target P: %.2f bar)\n", inletMotor.getStepCount(), currentPressureInlet, setPressureInlet);
    Serial.printf(" Discharge Steps: %ld (Current P: %.2f bar, Target P: %.2f bar)\n", dischargeMotor.getStepCount(), currentPressureDischarge, setPressureDischarge);
    Serial.printf(" Flow rate: %.2f (L/min)\n", currentFlow);
    Serial.printf(" Inlet motor enabled: %s\tDischarge motor enabled: %s\n",
                  inletMotor.isControlEnabled() ? "true" : "false", dischargeMotor.isControlEnabled() ? "true" : "false");
    Serial.printf(" Global reset flag: %s\n", reset ? "true" : "false");
    Serial.printf(" Inlet ADC Raw: %.2f, Discharge  ADC Raw: %.2f, Flow  ADC Raw: %.2f\n", inletFilo.average(), dischargeFilo.average(), flowFilo.average());
  } else if (serialStr == "reset") {
    Serial.println(" Initiating reset for both motors...");
    reset = true;  // Set global reset flag. managePressure will pick this up.

    // The following direct calls to managePressure and resetMotor are attempts to expedite
    // the reset process. The primary mechanism is the `reset` flag checked in `loop()`.
    Serial.println("Directly attempting Inlet motor reset sequence via managePressure...");
    inletMotor.managePressure(false, true);  // Ask managePressure to handle reset.
    Serial.println("Directly attempting Discharge motor reset sequence via managePressure...");
    dischargeMotor.managePressure(false, true);

    Serial.println("Re-attempting reset via direct blocking calls:");
    bool inletResetDone = inletMotor.resetMotor();  // Direct, blocking call.
    if (inletResetDone) {
      Serial.println("Inlet motor reset sequence reported success by direct call.");
    } else {
      Serial.println("Inlet motor reset sequence reported failure or timeout by direct call.");
    }

    bool dischargeResetDone = dischargeMotor.resetMotor();  // Direct, blocking call.
    if (dischargeResetDone) {
      Serial.println("Discharge motor reset sequence reported success by direct call.");
    } else {
      Serial.println("Discharge motor reset sequence reported failure or timeout by direct call.");
    }

    // If both blocking calls report success, clear the global reset flag.
    // Otherwise, it remains true, and `loop()` will continue trying to reset via `managePressure`.
    if (inletResetDone && dischargeResetDone) {
      Serial.println("Both motors reported reset completion by direct calls.");
      reset = false;
    } else {
      Serial.println("One or both motors failed to reset via direct calls. Global 'reset' flag remains true.");
    }
  } else if (serialStr == "config") {
    Serial.println("==== CONFIG (via Motor Objects where applicable) ====");
    Serial.printf("Max Steps - Inlet: %ld, Discharge: %ld\n", inletMotor.getMaxSteps(), dischargeMotor.getMaxSteps());
    Serial.printf("Inlet Target Pressure for Reset: %.2f\n", inletMotor._minPressureForReset);
    Serial.printf("Discharge Target Pressure for Reset: %.2f\n", dischargeMotor._minPressureForReset);
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
// Orphaned functions, logic moved/integrated elsewhere or removed if obsolete.
// void controlInletPressure() { /* ... */ }
// void controlDischargePressure() { /* ... */ }
