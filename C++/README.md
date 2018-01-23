# C++ Instructions

## Install
These steps assume you already have a working WPILib based C++ project working. If you are starting from scratch, see the [WPILib instructions](https://wpilib.screenstepslive.com/s/currentCS/m/cpp)
* Copy the `ADIS16448_IMU.cpp` file into your _cpp/_ folder
* Copy the `ADIS16448_IMU.h` file into your _include/_ folder

## Usage
Make a new instance of the driver, and use it however you like. For example:

```cpp
#include <ADIS16448_IMU.h>
#include <IterativeRobot.h>
#include "WPILib.h"

class Robot : public frc::IterativeRobot {

public:

	ADIS16448_IMU *imu;

	void RobotInit()
	{
		imu = new ADIS16448_IMU;
	}
	
	void TeleopPeriodic() {
    		SmartDashboard::PutData("IMU", imu);
    		Wait(0.005);
    	}
};
START_ROBOT_CLASS(Robot)

```
