# ADIS16448 RoboRIO LabVIEW Example for FIRST Robotics
### A working example built for FRC robots using the FRC LabVIEW development environment

This example library was written to give mentors, students, and engineers a starting point for using a very high-performance 10 Degree-of-Freedom (DoF), calibrated Inertial Measurement Unit (IMU). This sensor packages gyroscopes, accelerometers, magnetometers, and a barometer in a small, robust package perfect for high performance robotics (such as FRC). 

This software example offers:
- Raw sensor outputs - X-Y-Z Gyroscope, Accelerometer, Magnetometer, and Barometer
- X-Y-Z gyroscope angles calculated by means of loop integration
- AHRS (Pitch, Roll, Yaw) calculated using complementary and simplified Kalman (Madgwick) filters. More information can be found [here](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

Tutorial videos, how-to guides, and additional resources can be found at [http://www.analog.com/first](http://www.analog.com/first/)

**UPDATE 01/18/2017: Instruction on integrating the IMU driver into LabVIEW can be found [here](https://ez.analog.com/blogs/engineeringmind/2017/01/18/using-the-adis16448-imu-in-frc-labview).**

### What do I need to get started?

In order to use the software, you will need access to a RoboRIO and the ADIS16448 MXP Breakout Board. This software is based on the FRC 2017 LabVIEW software distribution, so previous versions may not work without correcting errors. 

Plug in the expansion board as shown below. **Be careful to not offset the connector!!** If installed correctly, the Power LED should turn on once power is applied to the RoboRIO.

![ADIS16448 Breakout Board Installed on a RoboRIO](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/IMG_5514.JPG)

### How do I install the library?

After cloning the repository, open the LabVIEW project `FRC ADI MXP IMU.lvproj` and double click on the example VI `FRC ADI MXP IMU Example.vi`. A front panel like the one shown below should appear once everything loads. Check your connection settings in the project and run the program. Instructions for integrating the IMU driver into LabVIEW can be found [here](https://ez.analog.com/blogs/engineeringmind/2017/01/18/using-the-adis16448-imu-in-frc-labview).

![ADIS16448 IMU Driver Front Panel](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/FrontPanel.png)

### Does the library support gyro drift calibration?

YES! The library now includes a calibration program! Open  `ADIS16448 IMU Calibration.vi` and follow the instructions on the GUI. Once the calibration program finishes, the calibration data will be stored in registers within the IMU. This compensation will be automatically applied to the sensor outputs during normal operation. If you would like to remove the calibration, select the "Clear Calibration" control and run the software. Note that running the calibration program more than once will overwrite the previous file.

If calibration data does not exist, the IMU will sample 3 seconds worth of data upon start-up and use this data to compensate for drift.

![ADIS16448 IMU Calibration Front Panel](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/Calibrate.PNG)

### A Shout-Out to the RoboBees
Thank you very much to Team 836, The RoboBees for providing the FIRST community with an excellent AHRS example! 
