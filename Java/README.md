# Java Instructions

## Gradle Install
As this driver depends on WPILib, we strongly recommend any team using gradle to use [GradleRIO](https://github.com/Open-RIO/GradleRIO)
* Add [JitPack](https://jitpack.io/#juchong/ADIS16448-RoboRIO-Driver) to your `repositories` block: ``maven { url 'https://jitpack.io' }``
* Add ``compile 'com.github.juchong:ADIS16448-RoboRIO-Driver:master-SNAPSHOT'`` to your `dependencies` block (this will always be the most recent version, see jitpack for more details)

Your gradle build file will end up looking something like this if you use GradleRIO: [build.gradle.example](build.gradle.example)

## Manual Install
* Copy the `com/analog/adis16448` package into your `src/` folder

## Usage
Make a new instance of the driver, and use it however you like. For example:
```java
import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {
  public static final ADIS16448_IMU imu = new ADIS16448_IMU();
  
  @Override
  public void robotPeriodic() { 
    SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
    SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
    SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());
    
    SmartDashboard.putNumber("Accel-X", imu.getAccelX());
    SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
    SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());
    
    SmartDashboard.putNumber("Pitch", imu.getPitch());
    SmartDashboard.putNumber("Roll", imu.getRoll());
    SmartDashboard.putNumber("Yaw", imu.getYaw());
    
    SmartDashboard.putNumber("Pressure: ", imu.getBarometricPressure());
    SmartDashboard.putNumber("Temperature: ", imu.getTemperature()); 
  }
}
```
