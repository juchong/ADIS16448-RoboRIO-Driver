## ***FRC TEAMS, BE AWARE THAT THE 2019 KICKOFF RELEASE IMAGE (v12) BREAKS COMPATIBILITY WITH ADI GYROS and IMUs! NI HAS RELEASED AN UPDATE SUITE (2019.1.0) ALONG WITH A NEW ROBORIO IMAGE (v13) THAT FIXES THE ISSUE. THE UPDATED INSTALLER CAN BE FOUND [HERE](http://www.ni.com/download/first-robotics-software-2017/7904/en/).***

# ADIS16448 IMU Library for FIRST Robotics and the RoboRIO

## Introduction
This example library was written to give mentors, students, and engineers a starting point for using a very high-performance 10 Degree-of-Freedom (DoF), calibrated Inertial Measurement Unit (IMU). This sensor packages gyroscopes, accelerometers, magnetometers, and a barometer in a small, robust package perfect for high performance robotics (such as FRC). 

These software libraries provide the user (you) with:
- Raw sensor outputs - X-Y-Z Gyroscope, Accelerometer, Magnetometer, and Barometer
- X-Y-Z gyroscope angles calculated by means of loop integration
- AHRS (Pitch, Roll, Yaw) calculated using complementary and simplified Kalman (Madgwick) filters. More information can be found [here](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)
- Offset compensation calculated at runtime

Tutorial videos, how-to guides, and additional resources can be found on the [ADI FIRST Robotics Wiki Page](https://wiki.analog.com/first/first_robotics_donation_resources).

## Why isn't your example code working?
A bug was introduced in the 2019 kickoff RoboRIO image which broke "Auto SPI." All ADI sensors currently offered to FRC teams rely on this feature to synchronously capture IMU data and feed it to the Zynq CPU for processing. The source of the bug was traced back to a RoboRIO image packaging issue and could be temporarily resolved on a v12 image by connecting to the serial/SSH terminal on the RoboRIO and executing the command "updateNIDrivers". This command forces the RoboRIO to re-compile the affected kernel module and fully resolves the issue. As of 01/17/2019, NI has released an updated [installer](http://www.ni.com/download/first-robotics-software-2017/7904/en/) (2019.1.0) that includes a pre-patched RoboRIO image (v13).

## What programming languages are supported?
The IMU driver currently supports all three official FRC languages (C++, Java, and LabVIEW). Raw sensor rate outputs, accumulated sensor outputs, and Kalman/Madgwick outputs are supported for all languages. 

## What do I need to get started?

In order to use the software, you will need access to a RoboRIO and the ADIS16448 MXP Breakout Board. This software is based on the FRC 2019 software distribution and relies on the WPILib libraries to interface with the IMU. Previous (pre-2019) versions of LabVIEW and WPILib libraries are **not** supported. 

Plug in the expansion board as shown below. **Be careful not to offset the connector!!** If installed correctly, the Power LED should turn on once the RoboRIO is powered on.

Your RoboRIO should be imaged to match the version of the NI Update Suite installed on your PC. For example, if you have the latest (of this writing) update suite installed (2019.0.0), then you must also have the **FRC_roboRIO_2019_v12** image and **roboRIO_6.0.0** firmware installed. This driver relies heavily on the FPGA image loaded in the RoboRIO and _**will not work**_ on older versions. The most current NI Update Suite can be found [here](https://forums.ni.com/t5/FIRST-Robotics-Competition/FRC-Update-Suite/ta-p/3737502).

![ADIS16448 Breakout Board Installed on a RoboRIO](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/IMG_5514.JPG)

## How do I use the IMU with my programming language?

Click on the language you're looking to use above. Each folder includes instructions specific to the language specified. If you're looking for more information on using the sensor, be sure to check out the [ADI FIRST Robotics Wiki Page](https://wiki.analog.com/first/first_robotics_donation_resources).

## Can I order my own PCB? Where can I find the schematic?

The schematic, layout, and manufacturing files can be found in this repository under `Reference/PCB Reference Files/`. 
Copies of this board may be purchased from OSH Park using this [link](https://oshpark.com/shared_projects/G8nquDEx). 

## A Shout-Out to the RoboBees

Thank you very much to Team 836, The RoboBees for providing the FIRST community with an excellent AHRS example!
