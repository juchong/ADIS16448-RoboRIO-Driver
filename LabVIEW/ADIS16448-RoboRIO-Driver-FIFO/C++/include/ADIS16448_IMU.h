/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "DigitalSource.h"
#include "GyroBase.h"
#include "InterruptableSensorBase.h"
#include "SPI.h"
#include "HAL/cpp/priority_mutex.h"

#include <atomic>
#include <cstdint>
#include <thread>

/**
 * This class is for the ADIS16448 IMU that connects to the RoboRIO MXP port.
 */
class ADIS16448_IMU : public GyroBase {
 public:
  ADIS16448_IMU();
  ~ADIS16448_IMU();

  void Calibrate() override;
  void Reset() override;
  float GetAngle() const override;
  double GetRate() const override;
  double GetAngleX() const;
  double GetAngleY() const;
  double GetAngleZ() const;
  double GetRateX() const;
  double GetRateY() const;
  double GetRateZ() const;
  double GetAccelX() const;
  double GetAccelY() const;
  double GetAccelZ() const;
  double GetMagX() const;
  double GetMagY() const;
  double GetMagZ() const;
  double GetPitch() const;
  double GetRoll() const;
  double GetYaw() const;
  double GetLastSampleTime() const;
  double GetBarometricPressure() const;
  double GetTemperature() const;
  double GetQuaternionW() const;
  double GetQuaternionX() const;
  double GetQuaternionY() const;
  double GetQuaternionZ() const;

  void UpdateTable();

 private:
  uint16_t ReadRegister(uint8_t reg);
  void WriteRegister(uint8_t reg, uint16_t val);
  void Calculate();

  // gyro center
  double m_gyro_center_x = 0.0;
  double m_gyro_center_y = 0.0;
  double m_gyro_center_z = 0.0;

  // last read values (post-scaling)
  double m_gyro_x = 0.0;
  double m_gyro_y = 0.0;
  double m_gyro_z = 0.0;
  double m_accel_x = 0.0;
  double m_accel_y = 0.0;
  double m_accel_z = 0.0;
  double m_mag_x = 0.0;
  double m_mag_y = 0.0;
  double m_mag_z = 0.0;
  double m_baro = 0.0;
  double m_temp = 0.0;

  // accumulated gyro values (for offset calculation)
  int m_accum_count = 0;
  double m_accum_gyro_x = 0.0;
  double m_accum_gyro_y = 0.0;
  double m_accum_gyro_z = 0.0;

  // integrated gyro values
  double m_integ_gyro_x = 0.0;
  double m_integ_gyro_y = 0.0;
  double m_integ_gyro_z = 0.0;

  // last sample time
  double m_last_sample_time = 0.0;

  // Kalman (AHRS)
  double m_ahrs_q1 = 1, m_ahrs_q2 = 0, m_ahrs_q3 = 0, m_ahrs_q4 = 0;
  double m_yaw = 0.0;
  double m_roll = 0.0;
  double m_pitch = 0.0;

  std::atomic_bool m_freed;

  SPI m_spi;
  uint8_t m_cmd[26], m_resp[26];
  std::unique_ptr<DigitalSource> m_interrupt;

  std::thread m_task;

  mutable priority_mutex m_mutex;
};
