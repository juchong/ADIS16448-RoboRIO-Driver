# ADIS16448 IMU Interface Library for FIRST Robotics

## Introduction

These libraries allow mentors, students, and engineers to quickly get started using the ADIS16448 Inertial Measurement Unit (IMU). This compact module provides teams with a high-performance, ten-degree-of-freedom (DoF), calibrated, feedback for their FRC robots. The module packages several gyroscopes and accelerometers in a small, robust package, perfect for high-performance robotics (such as FRC). 

<p align="center">
  <img src="https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/docs/448_IMU.jpg" alt="ADIS16448 Breakout Board for FRC" width="60%%" />
</p>

To simplify the library as much as possible for the 2020 season, only a few, key IMU features are exposed to the user by default. 

These software libraries provide the user (you) with:

- X, Y, and Z (Pitch, Roll, and Yaw) Integrated Gyroscope Outputs (user-selectable) calculated from 16-bit register outputs
- 16-bit X, Y, and Z instantaneous sensor outputs
- IMU register reads and writes using discrete SPI transactions and the Auto SPI peripheral built into the FRC 2020 RoboRIO image
- IMU heading initialization and continuous bias compensation routines
- Complementary filter outputs useful for measuring X-Y tilt

Tutorial videos, how-to guides, and additional resources can be found on the [ADI FIRST Robotics Wiki Page](https://wiki.analog.com/first/first_robotics_donation_resources).

## What programming languages are supported?

The IMU driver currently supports all three official FRC languages (C++, Java, and LabVIEW). LabVIEW libraries should be installed using the NI Package Manager.  C++ and Java libraries should be installed by copying the source files into your VSCode project, or by using maven.

## What do I need to get started?

To use the software, you need access to a RoboRIO and the ADIS16448 RoboRIO Breakout Board. This software is based on the FRC 2020 software distribution and relies on the latest WPILib libraries and RoboRIO image to interface with the IMU. Previous (pre-2020) versions of LabVIEW and WPILib libraries are **not** supported. 

Plug in the expansion board as shown below. **Be careful not to offset the connector!!** If installed correctly, the Power LED should turn on once the RoboRIO is powered on. If your board has a status LED, it will only light up once the IMU has successfully communicated with the RoboRIO.

**Your RoboRIO should be imaged to match the version of the NI Update Suite installed on your PC.** For example, if you have the latest (of this writing) update suite installed (2020.0.0), then you must also have the **FRC_roboRIO_2020_v10** image and **roboRIO_6.0.0f1** firmware installed. This driver relies heavily on the FPGA image loaded in the RoboRIO and _**will not work**_ on older versions. The most current NI Update Suite can be found [here](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#333285).

![ADIS16448 Breakout Board Installed on a RoboRIO](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/docs/IMG_5514.JPG)

## How do I use the IMU with my programming language?

Click on the language you're looking to use above. Each folder includes instructions specific to the language specified. If you're looking for more information on using the sensor, be sure to check out the [ADI FIRST Robotics Wiki Page](https://wiki.analog.com/first/first_robotics_donation_resources).

## Can I order my own PCB? Where can I find the schematic?

The schematic, layout, and manufacturing files can be found in this repository under `hardware/`. 
Copies of this board may be purchased from OSH Park using this [link](https://oshpark.com/shared_projects/G8nquDEx). 
