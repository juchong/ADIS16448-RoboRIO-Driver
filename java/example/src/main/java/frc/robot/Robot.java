/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.analog.adis16470.frc.ADIS16470_IMU;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  private static final String kYawDefault = "Z-Axis";
  private static final String kYawXAxis = "X-Axis";
  private static final String kYawYAxis = "Y-Axis";
  private String m_yawSelected;
  private ADIS16470_IMU.IMUAxis m_yawActiveAxis;
  private final SendableChooser<String> m_yawChooser = new SendableChooser<>();

  private boolean m_runCal = false;
  private boolean m_configCal = false;
  private boolean m_reset = false;
  private boolean m_setYawAxis = false;

  private final ADIS16470_IMU m_imu = new ADIS16470_IMU();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_autoChooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_autoChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_autoChooser);
    m_yawChooser.setDefaultOption("Z-Axis", kYawDefault);
    m_yawChooser.addOption("X-Axis", kYawXAxis);
    m_yawChooser.addOption("Y-Axis", kYawYAxis);
    SmartDashboard.putData("IMUYawAxis", m_yawChooser);

    SmartDashboard.putBoolean("RunCal", false);
    SmartDashboard.putBoolean("ConfigCal", false);
    SmartDashboard.putBoolean("Reset", false);
    SmartDashboard.putBoolean("SetYawAxis", false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("YawAngle", m_imu.getAngle());
    SmartDashboard.putNumber("XCompAngle", m_imu.getXComplementaryAngle());
    SmartDashboard.putNumber("YCompAngle", m_imu.getYComplementaryAngle());
    m_runCal = SmartDashboard.getBoolean("RunCal", false);
    m_configCal = SmartDashboard.getBoolean("ConfigCal", false);
    m_reset = SmartDashboard.getBoolean("Reset", false);
    m_setYawAxis = SmartDashboard.getBoolean("SetYawAxis", false);
    m_yawSelected = m_yawChooser.getSelected();

    // Set IMU settings
    if (m_configCal) {
      m_imu.configCalTime(ADIS16470_IMU.ADIS16470CalibrationTime._8s);
      m_configCal = SmartDashboard.putBoolean("ConfigCal", false);
    }
    if (m_reset) {
      m_imu.reset();
      m_reset = SmartDashboard.putBoolean("Reset", false);
    }
    if (m_runCal) {
      m_imu.calibrate();
      m_runCal = SmartDashboard.putBoolean("RunCal", false);
    }
    
    // Read the desired yaw axis from the dashboard
    if (m_yawSelected == "X-Axis") {
      m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kX;
    }
    else if (m_yawSelected == "Y-Axis") {
      m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kY;
    }
    else {
      m_yawActiveAxis = ADIS16470_IMU.IMUAxis.kZ;
    }
    // Set the desired yaw axis from the dashboard
    if (m_setYawAxis) {
      m_imu.setYawAxis(m_yawActiveAxis);
      m_setYawAxis = SmartDashboard.putBoolean("SetYawAxis", false);
    }

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_autoChooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
