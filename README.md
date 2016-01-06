# ADIS16448 RoboRIO LabVIEW Example for 2016 FIRST Robotics
### A working example built for FRC robots using the 2016 LabVIEW development environment

This example library was written to give mentors, students, and engineers a starting point for using a very high-performance 10 Degree-of-Freedom (DoF), calibrated Inertial Measurement Unit (IMU). This sensor packages gyroscopes, accelerometers, magnetometers, and a barometer in a small, robust package perfect for high performance robotics (such as FRC). 

This software example offers:
- Raw sensor outputs - X-Y-Z Gyroscope, Accelerometer, Magnetometer, and Barometer
- X-Y-Z gyroscope angles calculated by means of loop integration
- AHRS (Pitch, Roll, Yaw) calculated using a simplified Kalman (Madgwick) filter. More information can be found [here](http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

Tutorial videos, how-to guides, and additional resources can be found at [http://www.analog.com/first](http://www.analog.com/first/)

### What do I need to get started?

In order to use the software, you will need access to a RoboRIO and the ADIS16448 MXP Breakout Board. This software is based on the FRC 2016 LabVIEW software distribution, so previous versions may not work without correcting errors. 

Plug in the expansion board as shown below. **Be careful to not offset the connector!!** If installed correctly, the Power LED should turn on once power is applied to the RoboRIO.

![ADIS16448 Breakout Board Installed on a RoboRIO](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/IMG_5514.JPG)

### How do I install the library?

After cloning the repository, open the LabVIEW project `FRC ADI MXP IMU.lvproj` and double click on the example VI `FRC ADI MXP IMU Example.vi`. A front panel like the one shown below should appear once everything loads. Check your connection settings in the project and run the program. 

![ADIS16448 IMU Driver Front Panel](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/FrontPanel.PNG)

Once the LabVIEW code starts, **don't touch the RoboRIO!** The code records sensor data when starting up and uses it to correct for sensor drift. Once data appears in the indicators, the sensor is ready for use! 
