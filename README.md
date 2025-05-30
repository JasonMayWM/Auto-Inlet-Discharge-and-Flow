# Auto-Inlet-Discharge-and-Flow

This project provides firmware for a Raspberry Pi Pico-based control system that automatically regulates inlet pressure, discharge pressure, and flow rate using stepper motors and analog sensor feedback. It is designed for precision fluid handling applications such as pump testing, calibration systems, and automated lab setups. The system supports serial communication for remote control and data streaming.

---

## Features

- Dual-core operation for real-time control and communication
- PID-like pressure and flow regulation using stepper motors
- Acceleration and deceleration profiles for smooth motor control
- Serial command interface for remote pressure/flow control and monitoring
- Real-time analog sensor averaging for stable feedback
- Simulated delayed sensor response for testing
- Supports system start/stop, pressure and flow setpoints, and command streaming

---

## Hardware Requirements

- **Microcontroller**: Raspberry Pi Pico
- **Stepper Drivers**: TB6600 stepper motor drivers
- **Stepper Motors**: NEMA 17 with 51:1 gearboxes (for high torque and precision)
- **Sensors**:
  - Analog pressure sensors (inlet and discharge)
  - Analog flow sensor
- **Power Supply**: Suitable for stepper motors and TB6600 drivers
- **Miscellaneous**:
  - Breadboard or custom PCB
  - Serial interface (USB to UART or Pico USB port)

---

## Getting Started

1. **Clone the repository**  
   ```bash
   git clone https://github.com/JasonMayWM/Auto-Inlet-Discharge-and-Flow.git

