# Java Instructions

## Install
### Online Install
- Open Visual Studio Code
- Click the WPILib command pallete icon
- Select Manage Vendor Libraries
- Choose Install new libraries (online)
- Paste the following link: [http://maven.highcurrent.io/vendordeps/ADIS16448.json](http://maven.highcurrent.io/vendordeps/ADIS16448.json)

### Offline Install
- Download the latest release zip from the [releases](https://github.com/juchong/ADIS16448-RoboRIO-Driver/releases) page on Github. The zip will be named `adis16448_roborio-[releaseversion].zip`.
- Extract this zip to `~/home/frc2019/` (on windows `~/home` is `C:\Users\Public` so extract it to `C:\Users\Public\frc2019`).
- If Visual Studio code is open close the current folder (File > Close Folder) the exit Visual Studio Code.
- Open Visual Studio Code
- Click the SPILb command pallete icon
- Select Manage Vendor Libraries
- Choose Install new libraries (offline)
- Check ADIS16448 the click OK.

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
