# ADIS16448 IMU Library for FIRST Robotics and the RoboRIO

## Introduction
This example library was written to give mentors, students, and engineers a starting point for using a very high-performance 10 Degree-of-Freedom (DoF), calibrated Inertial Measurement Unit (IMU). This sensor packages gyroscopes, accelerometers, magnetometers, and a barometer in a small, robust package perfect for high performance robotics (such as FRC). 

These software libraries provide the user (you) with:
- Raw sensor outputs - X-Y-Z Gyroscope, Accelerometer, Magnetometer, and Barometer
- X-Y-Z gyroscope angles calculated by means of loop integration
- AHRS (Pitch, Roll, Yaw) calculated using complementary and simplified Kalman (Madgwick) filters. More information can be found [here](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

Tutorial videos, how-to guides, and additional resources can be found at [http://www.analog.com/first](http://www.analog.com/first/)

## What programming languages are supported?
The IMU driver currently supports all three official FRC languages (C++, Java, and LabVIEW). Raw sensor rate outputs, accumulated sensor outputs, and Kalman/Madgwick outputs are supported for all languages. 

**New for 2018:** Recent updates to the 2018 FRC libraries have dramatically improved sensor performance and greatly reduced CPU load!

## What do I need to get started?

In order to use the software, you will need access to a RoboRIO and the ADIS16448 MXP Breakout Board. This software is based on the FRC 2018 software distribution and relies on the WPILib libraries to interface with the IMU. Previous (pre-2018) versions of LabVIEW and WPILib libraries are **not** supported.

Plug in the expansion board as shown below. **Be careful to not offset the connector!!** If installed correctly, the Power LED should turn on once power is applied to the RoboRIO.

Your RoboRIO should be imaged to match the version of the NI Update Suite installed on your PC. For example, if you have the latest (of this writing) update suite installed (2018.1.0), then you must have the RoboRIO v17 image installed as well. This driver relies heavily on the FPGA image loaded in the RoboRIO and _**will not work**_ on older versions. The most up-to-date NI Update Suite can be found [here](https://forums.ni.com/t5/FIRST-Robotics-Competition/FRC-Update-Suite/ta-p/3737502).

![ADIS16448 Breakout Board Installed on a RoboRIO](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/IMG_5514.JPG)

## How do I use the IMU with my programming language?

Click on the language you're looking to use above. Each folder includes instructions specific to the language specified. 

## A Shout-Out to the RoboBees

Thank you very much to Team 836, The RoboBees for providing the FIRST community with an excellent AHRS example! 
