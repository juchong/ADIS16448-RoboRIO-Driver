LabVIEW Instructions

## Self-Extracting Installation
This driver assumes you have a working LabVIEW installation and 2018.0+ FRC software package already installed on your system. The self-extracting installer can be downloaded from the releases page [here](https://github.com/juchong/ADIS16448-RoboRIO-Driver/releases). 

Once downloaded, run the installer with administrator privileges. All the required files will be extracted to integrate the IMU driver with LabVIEW. Examples will also be installed to make testing your sensor easy!

## Manual Installation
Copy the `ADIS16448 IMU` folder into the folder where your LabVIEW robot project is stored. Once copied, drag the `ADIS16448 IMU` folder into the project explorer. 

## Usage
The LabVIEW driver can be used like any other sensor. If the **installer** was used, all .vi's can be found in the pallet shown below:

![ADIS16448 LabVIEW Pallet](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/pallet.PNG)

If the driver was added to your project **manually**, all user .vi's should be located in your project like this:

![ADIS16448 LabVIEW Project Explorer](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/RobotProject.PNG)

## Examples
If the **installer** was used, the examples can be accessed by following the steps below:

Navigate to `Help > Find Examples...` in the main window.

![ADIS16448 Main Menu](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/MainMenu.PNG)

In the NI Example Finder, navigate to `FRC Robotics > Sensors > ADIS16448 IMU.lvproj` and double click. 

![ADIS16448 Find Examples](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/ExampleFinder.PNG)

The example project will open. Select `ADIS16448 IMU Example.vi` and run the example. If the sensor was successfully detected, the front panel should look similar to this:

![ADIS16448 Front Panel](https://raw.githubusercontent.com/juchong/ADIS16448-RoboRIO-Driver/master/Reference/FrontPanel.png)

