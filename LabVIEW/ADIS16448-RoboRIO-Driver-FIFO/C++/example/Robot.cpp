#include "WPILib.h"

#include "ADIS16448_IMU.h"

class Robot: public SampleRobot
{
private:
    ADIS16448_IMU *imu;

    void RobotInit()
    {
        imu = new ADIS16448_IMU;
    }

    void Autonomous()
    {
    }

    void OperatorControl()
    {
        while (IsOperatorControl() && IsEnabled()) {
            SmartDashboard::PutData("IMU", imu);
            Wait(0.005);
        }
    }

    void Test()
    {
    }
};

START_ROBOT_CLASS(Robot)
