# WindTunnelSuite

Control and monitoring system for a small educational wind tunnel built around **Arduino** and a **Windows Forms PC application**.

This project combines:

- a physical wind tunnel prototype
- Arduino firmware for sensors, fans, lighting, and airfoil-angle control
- a Windows desktop UI for live control and telemetry
- KiCad schematic files
- 3D printed mechanical parts for the enclosure, tunnel sections, venturi iterations, and smoke-flow visualization parts

---

## Project purpose

The goal of this project is to create a **modular and understandable wind tunnel system** suitable for:

- school demonstrations
- basic aerodynamic experiments
- testing airflow behavior around an airfoil
- learning embedded control, sensors, PWM, serial communication, and PC-side visualization

The current implementation is intentionally built at **educational / demonstrational scale**.  
The present fan solution uses **4 smaller fans** as a modular temporary approach, but the design can be adapted later to a **single larger high-flow fan**.

---

## Main features

### Arduino side
- PWM control of multiple fans
- control of airfoil angle through a stepper motor and A4988 driver
- angle feedback with AS5600 magnetic encoder
- barometric pressure and temperature measurement
- differential pressure measurement for airflow-related testing
- serial command parser for communication with the PC app
- telemetry output back to the PC application
- EEPROM storage for encoder zero / home behavior

### PC application side
- serial COM port connection
- separate UI pages for:
  - fan control
  - sensor live data
  - airfoil angle control
- live telemetry parsing
- console log for TX/RX debugging
- visual organization of controls for easy demonstration use

### Mechanical side
- 3D printed wind tunnel body and sections
- two physical design directions / configurations were used during development:
  - larger concept around **540 x 540 mm** inlet class
  - smaller demonstrational configuration around **240 x 240 mm** for use with PC fans
- venturi development through multiple printed variants
- smoke injector and holder for visual airflow observation
- printed enclosure / holder components for electronics and mounting

---

## Repository contents

Typical contents in this repository include:

- Arduino sketch / firmware
- Windows Forms solution
- KiCad schematic files
- schematic export PDF
- 3D model files (`.3mf`)
- supporting documentation and project assets

Important project files include:

- `WTS_Arduino_Sketch.ino` – Arduino firmware
- `WindTunnelSuite.sln` – Visual Studio solution
- `Arduino LED Blinker/WindTunnelSuite.csproj` – WinForms project
- `Arduino LED Blinker/Form1.cs` – main shell / navigation / serial handling
- `Arduino LED Blinker/FansPage.xaml.cs` – fan control page
- `Arduino LED Blinker/SensorsPage.xaml.cs` – live sensor page
- `Arduino LED Blinker/AirfoilAnglePage.cs` – airfoil angle control page
- `Arduino LED Blinker/TelemetryStore.cs` – telemetry parser / storage
- KiCad files for the electrical design
- `.3mf` files for the mechanical parts

---

## Hardware overview

### Main controller
- **Arduino Uno**  
Chosen because it is widely available, easy to program, well documented, and fully adequate for a school-scale control system.

### Sensors
- **AS5600 magnetic encoder**  
Used to measure the real angular position of the airfoil mechanism.

- **MPXV7002 differential pressure sensor**  
Used for airflow / pressure-difference observation in the test section.

- **BMP sensor module**  
The project wiring was prepared so the connector can support a **BMP280-style module**, while the current code path can also work with a **BMP180/BMP085-type module** because only the required **3.3 V, GND, SDA, and SCL** lines are needed in this operating mode.

### Actuators
- **A4988 stepper driver + NEMA 17 stepper motor**  
Used to rotate and position the airfoil.

- **4 fans**  
Used in the current prototype for demonstrational airflow generation. This is a modular starter solution.

- **LED lighting via MOSFET**  
Used for visual illumination of the test area.

### Protection / support components
- fuse on the AC input side
- onboard / supply-side protection
- filter / buffer capacitors
- gate resistor and pull-down resistor for MOSFET-based switching
- common ground strategy for correct signal reference

### Capacitors
Capacitors in the system are used mainly as **parallel filtering and buffering elements** on the supply rails.  
Their role is to:
- reduce voltage dips
- smooth disturbances caused by switching loads
- improve stability when motors, fans, or other loads change state

Examples used in the project documentation:
- **1000 µF / 25 V**
- **100 µF / 25 V**

---

## Electrical connection summary

### I2C bus
The I2C sensor modules are connected **in parallel** to the same:
- SDA line
- SCL line
- 3.3 V supply
- ground

This is normal I2C bus wiring and allows multiple compatible devices on the same bus.

### Fans
The fans are powered as **parallel loads** from the common supply, while their control is handled through PWM-related circuitry / control outputs.

### LED lighting
The LED lighting is treated as a **parallel-powered load** controlled through a MOSFET switching stage.

### AC input path
The AC switch and fuse are connected **in series** in the mains input path so that:
- the switch can disconnect power
- the fuse can interrupt the circuit in fault conditions

### Potentiometer
The potentiometer is used as a **voltage divider**.  
Its output is read by an Arduino analog input and converted into a control value.

---

## Software architecture

### Arduino firmware
The firmware is structured around several responsibilities:

- reading commands from serial
- controlling fans
- controlling the stepper motor
- reading sensors
- outputting telemetry
- managing home / zero logic for the encoder

Typical command ideas used by the PC app include:

- `FAN ALL <value>`
- `FAN 1 <value>`
- `FAN 2 <value>`
- `FAN 3 <value>`
- `FAN 4 <value>`
- `FAN STOP`
- `STEPPER GOTO <steps> <speed>`
- `STEPPER STOP`
- `ENCODER ZERO`

Telemetry is sent back in a compact key-value style so the desktop app can parse and display live values.

### Notes
- In the current development stage, some RPM values shown in the software can be **simulated from PWM percentage** instead of coming from real tachometer feedback.
- Telemetry names may reflect earlier naming choices; for example a field like `mpx_adc` may actually represent a processed pressure-related value in software rather than a raw ADC count.

### Windows Forms application

The desktop software is built with **C# Windows Forms**.

#### Main responsibilities
- connect to Arduino through a COM port
- send user commands
- parse incoming telemetry
- display live values
- show a TX/RX console for debugging

#### Main pages

**Fans page**  
Used to control:
- all fans together
- each fan individually

It also displays RPM-style feedback values for demonstration.

**Sensors page**  
Displays live readings such as:
- last packet received
- fan-related values
- differential pressure
- barometric pressure
- temperature
- encoder angle

**Airfoil Angle page**  
Used to:
- set target angle
- control movement speed
- stop motion
- home the mechanism
- zero the encoder reference

#### UI helper components
A custom `GlassPanel` is used to give the interface a cleaner demonstrational layout.

---

## Libraries

The exact Arduino libraries may depend on the current firmware revision, but the project uses or has used libraries in this area:

- `Wire`
- `EEPROM`
- `AccelStepper`
- Adafruit BMP sensor library path such as:
  - `Adafruit_BMP085` for BMP180/BMP085-style support

Check the `.ino` file for the exact current include list used in the active version.

---

## Real-world testing

The project has been physically tested and documented with:

- real hardware prototype photos
- oscilloscope captures of fan PWM behavior
- fan PWM duty-cycle checks
- sensor checks
- repeated startup / restart behavior
- stepper movement evaluation
- physical airfoil positioning

Oscilloscope photos in the project/documentation correspond to **fan PWM testing** at different duty cycles such as:
- 0%
- 25%
- 50%
- 75%
- 100%

---

## How to build / run

### Arduino
1. Open the Arduino sketch.
2. Install the required libraries.
3. Select the correct board and COM port.
4. Upload the firmware to the Arduino.

### Windows app
1. Open `WindTunnelSuite.sln` in Visual Studio.
2. Build the solution.
3. Run the WinForms application.
4. Select the correct COM port.
5. Connect and begin testing.

---

## Suggested usage flow

1. Power the hardware safely.
2. Connect the Arduino to the PC.
3. Open the Windows application.
4. Select the COM port and connect.
5. Test fan control.
6. Test sensor telemetry.
7. Test airfoil angle movement and zeroing.
8. Observe airflow behavior and, where used, smoke visualization.

---

## Current status

This project is already a **real physical prototype**, not only a theoretical concept.

It includes:
- electrical design
- embedded control
- PC interface
- 3D printed mechanical development
- test measurements
- educational demonstrational functionality

At the same time, it remains open for future upgrades such as:
- larger fan / blower solution
- more accurate airflow measurement
- real tachometer RPM feedback for all fans
- automatic calibration routines
- expanded data logging
- improved aerodynamic measurement accessories

---

## Safety notes

This project includes mains-powered and motor-driven elements.  
Use caution when testing:

- protect AC input properly
- ensure proper insulation
- verify polarity and voltage levels before powering sensors
- do not hot-plug sensitive modules unless the design explicitly supports it
- keep a common ground where required for logic-level reference
- mechanically secure all moving and powered parts

---

## License / usage

This repository was created as part of an engineering / educational diploma project.  
Reuse is possible for learning and experimental purposes, but always verify electrical safety and component suitability for your own build.

---

## Author

**Tony Shivenkov**  
Applied programming diploma project  
Wind tunnel control system
