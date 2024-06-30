# Embedded Systems Project - system for an autonomous ground vehicle

## Project Overview
The project implements a basic control system for an autonomous ground vehicle. The system is designed to control the movement of the vehicle, process commands, handle motor control, sense obstacles, and manage power efficiently.

## Features
- **Control Loop:** Implemented at 1 kHz frequency for precise control.
- **Motor Control:** PWM signals control the motors to drive the vehicle in various directions.
- **Command Processing:** Commands are received and executed in a FIFO order.
- **Obstacle Detection:** The vehicle maintains a safe distance from obstacles.
- **Data Logging:** Battery voltage and distance data are logged and sent via UART.

## Firmware description
### Main Loop description
- Control loop frequency: 1 kHz
- Motor PWM update frequency: 1 kHz
- IR sensor read frequency: 1 kHz
- Initial state: "Wait for start"
  - PWM DC of all motors: 0
  - LED A0 and indicators: Blink at 1 Hz
- "Execute" state on button RE8 press
  - Process commands in FIFO order
  - Commands format: `$PCCMD,x,t*` where `x` is action and `t` is duration in ms
  - Actions:
    - 1 = Forward motion
    - 2 = Counterclockwise rotation
    - 3 = Clockwise rotation
    - 4 = Backward motion
  - FIFO queue for commands: Max 10 commands
  - Stop on empty FIFO, maintaining "executing" state
  - Obstacle detection: Stop within 20 cm


### Motor Control
- PWM signals on pins RD1 to RD4 at 10 kHz frequency
- Wheel actuation based on command:
  - Left wheels forward: RD2 = `left_pwm`, RD1 = 0
  - Left wheels backward: RD1 = `left_pwm` (negative), RD2 = 0
  - Right wheels forward: RD4 = `right_pwm`, RD3 = 0
  - Right wheels backward: RD3 = `right_pwm` (negative), RD4 = 0

### Battery Sensing
- Voltage sensing on pin AN11 using a partitioning circuit

### IR Sensor
- Mounted on Buggy Mikrobus 1 or 2
- Signal read on AN14/AN15, enable on RB9/RA3

### Data Logging / Command Interface through UART
- UART to RS232 module on Clicker Mikrobus 2
- TX signal: RD0/RP64
- RX signal: RD11/RPI75
- Messages to PC:
  - `$MBATT,v_batt*` (battery voltage, 1 Hz)
  - `$MDIST,distance*` (distance, 10 Hz)

## Pin Mapping
- RB8: Left side lights
- RF1: Right-side lights
- RF0: Brakes
- RG1: Low intensity lights
- RA7: Beam headlights
- AN11: Battery sensing
- RD1/RP65: Left PWM backward motion
- RD2/RP66: Left PWM forward motion
- RD3/RP67: Right PWM backward motion
- RD4/RP68: Right PWM forward motion
- AN14 or AN15: IR sensor voltage
- RB9 or RA3: IR sensor enable
- RD0/RP64: UART TX
- RD11/RPI75: UART RX


# The codes were written by:
- Ouassim Milous 
- Younes Hebik 
- Mohamed Aimen BOULALA 

License
This project is licensed under the MIT License. See the LICENSE file for more details.

# Acknowledgements
The project was suggested by professor Enrico Simetti and was a part of his course of embedded systems for Robotics Engineering Master's for the University of Genoa

Special thanks to the professor and all contributors and supporters.
