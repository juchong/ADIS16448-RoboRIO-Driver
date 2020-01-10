/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_autoChooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_autoChooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  m_yawChooser.SetDefaultOption(kYawDefault, kYawDefault);
  m_yawChooser.AddOption(kYawXAxis, kYawXAxis);
  m_yawChooser.AddOption(kYawYAxis, kYawYAxis);
  frc::SmartDashboard::PutData("Auto Modes", &m_autoChooser);
  frc::SmartDashboard::PutData("IMUYawAxis", &m_yawChooser);
  frc::SmartDashboard::PutBoolean("RunCal", false);
  frc::SmartDashboard::PutBoolean("ConfigCal", false);
  frc::SmartDashboard::PutBoolean("Reset", false);
  frc::SmartDashboard::PutBoolean("SetYawAxis", false);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functiosns, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("YawAngle", m_imu.GetAngle());
  frc::SmartDashboard::PutNumber("XCompAngle", m_imu.GetXComplementaryAngle());
  frc::SmartDashboard::PutNumber("YCompAngle", m_imu.GetYComplementaryAngle());
  m_runCal = frc::SmartDashboard::GetBoolean("RunCal", false);
  m_configCal = frc::SmartDashboard::GetBoolean("ConfigCal", false);
  m_reset = frc::SmartDashboard::GetBoolean("Reset", false);
  m_setYawAxis = frc::SmartDashboard::GetBoolean("SetYawAxis", false);
  m_yawSelected = m_yawChooser.GetSelected();

  // Set IMU settings
  if (m_configCal) {
    m_imu.ConfigCalTime(8);
    m_configCal = frc::SmartDashboard::PutBoolean("ConfigCal", false);
  }
  if (m_reset) {
    m_imu.Reset();
    m_reset = frc::SmartDashboard::PutBoolean("Reset", false);
  }
  if (m_runCal) {
    m_imu.Calibrate();
    m_runCal = frc::SmartDashboard::PutBoolean("RunCal", false);
  }
  
  // Read the desired yaw axis from the dashboard
  if (m_yawSelected == "X-Axis") {
    m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kX;
  }
  else if (m_yawSelected == "Y-Axis") {
    m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kY;
  }
  else {
    m_yawActiveAxis = frc::ADIS16448_IMU::IMUAxis::kZ;
  }
  // Set the desired yaw axis from the dashboard
  if (m_setYawAxis) {
    m_imu.SetYawAxis(m_yawActiveAxis);
    m_setYawAxis = frc::SmartDashboard::PutBoolean("SetYawAxis", false);
  }
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_autoChooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
