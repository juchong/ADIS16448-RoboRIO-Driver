
package example;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.analog.adis16448.frc.ADIS16448_IMU;

public class Robot extends SampleRobot {
    ADIS16448_IMU imu;

    public Robot() {
    }
    
    public void robotInit() {
        imu = new ADIS16448_IMU();
    }

    public void autonomous() {
    }

    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            SmartDashboard.putData("IMU", imu);
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

    public void test() {
    }
}
