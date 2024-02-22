# ARb Project
## Introduction
ARb is a robotic arm with 4 degrees of freedom. It is designed to be used in a variety of applications, including pick and place, etc. The arm is controlled by a host computer, which communicates with an STM32 to control the servos using UART via USB. The arm is also equipped with a sensor and a gripper, which can be used to pick up and manipulate objects. The arm is designed to be easy to use. The arm also designed to be affordable, with a total cost of less than Rp500.000,00. _Although we spent more because of trial and error, and we also bought some parts in a higher price than the market price._
## Features
- 4 degrees of freedom
- 1 gripper with integrated sensor
- Automatic find and pick up object
- Easy to use
- Fully programmable
## Components
- 3x MG996R Servo
- 2x MG90S Servo
- 1x VL53L0X Sensor
- 1x STM32F401CCU6
- 1x LM2596 DC-DC Step Down
- Custom PCB and Acrylic parts
## Tech Stack
- C -> STM32
    - HAL -> Hardware Abstraction Layer
    - VL53L0X -> Sensor
    - USB CDC -> Communication
- Python -> Host Computer
  - PySerial -> Communication
  - IKPy -> Inverse Kinematics
- Solidworks -> Design
- KiCad -> PCB Design
## Expenditure
[Rincian_Anggaran.pdf](https://github.com/hanzamzamy/arb-fpro/blob/master/Rincian_Anggaran.pdf)
## Documentation
[Video Demo.mp4](https://github.com/hanzamzamy/arb-fpro/blob/master/Dokumentasi/Video%20Demo.mp4)
[Robot.jpg](https://github.com/hanzamzamy/arb-fpro/blob/master/Dokumentasi/Barang%20e.jpg)
[Photo1.jpg](https://github.com/hanzamzamy/arb-fpro/blob/master/Dokumentasi/1.jpg)
[Photo2.jpg](https://github.com/hanzamzamy/arb-fpro/blob/master/Dokumentasi/1.jpg)
## Team
- Mechanical - [Silvia Nur Aziza]()
- Electrical - [Rayhan Rizqi Zamzamy](https://github.com/hanzamzamy)
- Programming - [Nabiel Nizar Anwari](https://github.com/bielnzar)
