# Controls Lab (EE324)
# EE324 Control Systems Lab - IIT Bombay

This repository contains reports, code, and other materials for the **Control Systems Lab (EE324)** course at **IIT Bombay**. The experiments focus on designing and implementing control systems for real-world applications using techniques such as PID control and LQR, implemented on an Arduino Mega. Below is a summary of the experiments conducted during the course.

## Experiments

### Experiment 1: DC Motor Position Control
- **Objective**: 
  - Design and implement a PID position controller to rotate a DC motor by 180 degrees.
  - The design must adhere to specifications such as:
    - **Rise Time**: 0.5 seconds
    - **Settling Time**: 1 second
    - **Overshoot**: Maximum 10%
  
- **Materials**: 
  - DC Motor setup, Arduino Mega, Power Supply, L293D IC, Jumper Wires, Breadboard, and other tools.
  
- **Key Topics**: 
  - PID control logic, Arduino programming
  
- **Important Files**:
  - `Experiment1_Report.pdf`: Detailed report and findings
  - `Experiment1_Code.ino`: Arduino code for the motor control
  
### Experiment 2: Inverted Pendulum Control
- **Objective**: 
  - Design and implement an LQR control system to maintain an inverted pendulum in an upright position while limiting:
    - **Pendulum arm vibration** (α): ±3 degrees
    - **Base angle oscillation** (θ): ±30 degrees
  
- **Materials**:
  - Inverted Pendulum setup, Arduino Mega, Decoder Shield, Jumper Wires, and other tools.

- **Key Topics**: 
  - LQR technique, State space modeling, Matlab and Arduino coding

- **Important Files**:
  - `Experiment2_Report.pdf`: Detailed report and findings
  - `Experiment2_Code.ino`: Arduino code for the LQR control system
  - `LQR.m`: MATLAB code where the LQR model is implemented to determine Optimal Gain Values
  - `simulink_model_continuous`: Simulink Model to emulate the inverted pendulum setup in a continuous time domain
  - `simulink_model_discrete`: Simulink Model to simulate the inverted pendulum similar to the control algorithm given that Arduino control happens in discrete time domain

## Repository Structure
```plaintext
├── Experiment1
│   ├── Experiment1_Report.pdf
│   ├── Experiment1_Code.ino
├── Experiment2
│   ├── Experiment2_Report.pdf
|   ├── Codes
│   |   ├── Experiment2_Code.ino
│   |   ├── LQR.m
│   |   ├── simulink_model_continuous.slx
│   |   ├── simulink_model_discrete.slx
│   ├── Problem_Statement.pdf
│   ├── Supporting_Documents
│   |   ├── amt22_datasheet.pdf
│   |   ├── L293D datasheet.pdf
│   |   ├── PendulumSetup_manual.pdf
└── README.md
```

## Pre-requisites
- **Software**:
  - Arduino IDE for programming and uploading code to the Arduino Mega.
  - MATLAB for state-space modeling and control design.
- **Knowledge**:
  - PID Control: Understanding proportional, integral, and derivative components in control systems.
  - LQR Technique: Knowledge of linear-quadratic regulator (LQR) and how it applies to control problems.
  - State-Space Modeling: Experience in developing state-space models for dynamic systems.  
  - Arduino Programming: Familiarity with Arduino IDE, coding, and interfacing with external hardware.
