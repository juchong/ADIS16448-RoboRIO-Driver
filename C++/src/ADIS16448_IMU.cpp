/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ADIS16448_IMU.h"

#include "LiveWindow/LiveWindow.h"
#include "DigitalInput.h"
#include "DriverStation.h"
#include "ErrorBase.h"
#include "Timer.h"
#include "WPIErrors.h"

#include <cmath>

static constexpr double kTimeout = 0.1;
static constexpr double kCalibrationSampleTime = 5.0;
static constexpr double kDegreePerSecondPerLSB = 1.0/25.0;
static constexpr double kGPerLSB = 1.0/1200.0;
static constexpr double kMilligaussPerLSB = 1.0/7.0;
static constexpr double kMillibarPerLSB = 0.02;
static constexpr double kDegCPerLSB = 0.07386;
static constexpr double kDegCOffset = 31;

static constexpr uint8_t kGLOB_CMD = 0x3E;
static constexpr uint8_t kRegSMPL_PRD = 0x36;
static constexpr uint8_t kRegSENS_AVG = 0x38;
static constexpr uint8_t kRegMSC_CTRL = 0x34;
static constexpr uint8_t kRegPROD_ID = 0x56;

static constexpr double kGyroScale = 0.0174533;   // rad/sec
static constexpr double kAccelScale = 9.80665;    // mg/sec/sec
static constexpr double kMagScale = 0.1;          // uTesla
static constexpr double kBeta = 1;

static inline uint16_t ToUShort(const uint8_t* buf) {
  return ((uint16_t)(buf[0]) << 8) | buf[1];
}

static inline int16_t ToShort(const uint8_t* buf) {
  return (int16_t)(((uint16_t)(buf[0]) << 8) | buf[1]);
}

using namespace frc;

/**
 * Constructor.
 */
ADIS16448_IMU::ADIS16448_IMU() : m_spi(SPI::Port::kMXP) {
  m_spi.SetClockRate(1000000);
  m_spi.SetMSBFirst();
  m_spi.SetSampleDataOnFalling();
  m_spi.SetClockActiveLow();
  m_spi.SetChipSelectActiveLow();

  ReadRegister(kRegPROD_ID); // dummy read

  // Validate the part ID
  if (ReadRegister(kRegPROD_ID) != 16448) {
    DriverStation::ReportError("could not find ADIS16448");
    return;
  }

  // Set IMU internal decimation to 102.4 SPS
  WriteRegister(kRegSMPL_PRD, 769);

  // Enable Data Ready (LOW = Good Data) on DIO1 (PWM0 on MXP)
  WriteRegister(kRegMSC_CTRL, 4);

  // Configure IMU internal Bartlett filter
  WriteRegister(kRegSENS_AVG, 1030);

  m_cmd[0] = kGLOB_CMD;
  m_cmd[1] = 0;

  // Configure interrupt on MXP DIO0
  m_interrupt.reset(new DigitalInput(10));
  m_interrupt->RequestInterrupts();
  m_interrupt->SetUpSourceEdge(false, true);

  // Start monitoring thread
  m_freed = false;
  m_task = std::thread(&ADIS16448_IMU::Calculate, this);

  Calibrate();

  //HALReport(HALUsageReporting::kResourceType_ADIS16448, 0);
  LiveWindow::GetInstance()->AddSensor("ADIS16448_IMU", 0, this);
}

/**
 * {@inheritDoc}
 */
void ADIS16448_IMU::Calibrate() {
  Wait(0.1);

  {
    std::lock_guard<priority_mutex> sync(m_mutex);
    m_accum_count = 0;
    m_accum_gyro_x = 0.0;
    m_accum_gyro_y = 0.0;
    m_accum_gyro_z = 0.0;
  }

  Wait(kCalibrationSampleTime);

  {
    std::lock_guard<priority_mutex> sync(m_mutex);
    m_gyro_center_x = m_accum_gyro_x / m_accum_count;
    m_gyro_center_y = m_accum_gyro_y / m_accum_count;
    m_gyro_center_z = m_accum_gyro_z / m_accum_count;
  }
}

uint16_t ADIS16448_IMU::ReadRegister(uint8_t reg) {
  uint8_t buf[2];
  buf[0] = reg & 0x7f;
  buf[1] = 0;

  m_spi.Write(buf, 2);
  m_spi.Read(false, buf, 2);

  return ToUShort(buf);
}

void ADIS16448_IMU::WriteRegister(uint8_t reg, uint16_t val) {
  uint8_t buf[2];
  buf[0] = 0x80 | reg;
  buf[1] = val & 0xff;
  m_spi.Write(buf, 2);
  buf[0] = 0x81 | reg;
  buf[1] = val >> 8;
  m_spi.Write(buf, 2);
}

/**
 * {@inheritDoc}
 */
void ADIS16448_IMU::Reset() {
  std::lock_guard<priority_mutex> sync(m_mutex);
  m_integ_gyro_x = 0.0;
  m_integ_gyro_y = 0.0;
  m_integ_gyro_z = 0.0;
}

/**
 * Delete (free) the spi port used for the IMU.
 */
ADIS16448_IMU::~ADIS16448_IMU() {
  m_freed = true;
  if (m_task.joinable()) m_task.join();
}

void ADIS16448_IMU::Calculate() {
  {
    std::lock_guard<priority_mutex> sync(m_mutex);
    m_last_sample_time = Timer::GetFPGATimestamp();
  }
  while (!m_freed) {
    if (m_interrupt->WaitForInterrupt(kTimeout) ==
        InterruptableSensorBase::WaitResult::kTimeout)
      continue;

    double sample_time = m_interrupt->ReadFallingTimestamp();
    double dt;
    {
      std::lock_guard<priority_mutex> sync(m_mutex);
      dt = sample_time - m_last_sample_time;
      m_last_sample_time = sample_time;
    }

    m_spi.Transaction(m_cmd, m_resp, 26);

    double gyro_x = ToShort(&m_resp[4]) * kDegreePerSecondPerLSB;
    double gyro_y = ToShort(&m_resp[6]) * kDegreePerSecondPerLSB;
    double gyro_z = ToShort(&m_resp[8]) * kDegreePerSecondPerLSB;
    double accel_x = ToShort(&m_resp[10]) * kGPerLSB;
    double accel_y = ToShort(&m_resp[12]) * kGPerLSB;
    double accel_z = ToShort(&m_resp[14]) * kGPerLSB;
    double mag_x = ToShort(&m_resp[16]) * kMilligaussPerLSB;
    double mag_y = ToShort(&m_resp[18]) * kMilligaussPerLSB;
    double mag_z = ToShort(&m_resp[20]) * kMilligaussPerLSB;

    // Make local copy of quaternion and angle global state
    double q1, q2, q3, q4;
    {
      std::lock_guard<priority_mutex> sync(m_mutex);
      q1 = m_ahrs_q1;
      q2 = m_ahrs_q2;
      q3 = m_ahrs_q3;
      q4 = m_ahrs_q4;
    }

    // Kalman calculation
    // Code originated from: https://decibel.ni.com/content/docs/DOC-18964
    do {
      // If true, only use gyros and magnetos for updating the filter.
      bool excludeAccel = false;

      // Convert accelerometer units to m/sec/sec
      double ax = accel_x * kAccelScale;
      double ay = accel_y * kAccelScale;
      double az = accel_z * kAccelScale;
      // Normalize accelerometer measurement
      double norm = std::sqrt(ax * ax + ay * ay + az * az);
      if (norm > 0.3 && !excludeAccel) {
        // normal larger than the sensor noise floor during freefall
        norm = 1.0 / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;
      } else {
        ax = 0;
        ay = 0;
        az = 0;
      }

      // Convert magnetometer units to uTesla
      double mx = mag_x * kMagScale;
      double my = mag_y * kMagScale;
      double mz = mag_z * kMagScale;
      // Normalize magnetometer measurement
      norm = std::sqrt(mx * mx + my * my + mz * mz);
      if (norm > 0.0) {
        norm = 1.0 / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;
      } else {
        break; // something is wrong with the magneto readouts
      }

      double _2q1 = 2.0 * q1;
      double _2q2 = 2.0 * q2;
      double _2q3 = 2.0 * q3;
      double _2q4 = 2.0 * q4;
      double _2q1q3 = 2.0 * q1 * q3;
      double _2q3q4 = 2.0 * q3 * q4;
      double q1q1 = q1 * q1;
      double q1q2 = q1 * q2;
      double q1q3 = q1 * q3;
      double q1q4 = q1 * q4;
      double q2q2 = q2 * q2;
      double q2q3 = q2 * q3;
      double q2q4 = q2 * q4;
      double q3q3 = q3 * q3;
      double q3q4 = q3 * q4;
      double q4q4 = q4 * q4;

      // Reference direction of Earth's magnetic field
      double _2q1mx = 2 * q1 * mx;
      double _2q1my = 2 * q1 * my;
      double _2q1mz = 2 * q1 * mz;
      double _2q2mx = 2 * q2 * mx;

      double hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
      double hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
      double _2bx = std::sqrt(hx * hx + hy * hy);
      double _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
      double _4bx = 2.0 * _2bx;
      double _4bz = 2.0 * _2bz;
      double _8bx = 2.0 * _4bx;
      double _8bz = 2.0 * _4bz;

      // Gradient descent algorithm corrective step
      double s1 =
        - _2q3 * (2.0 * q2q4 - _2q1q3 - ax)
        + _2q2 * (2.0 * q1q2 + _2q3q4 - ay)
        - _4bz * q3 * (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
        + (-_4bx * q4 + _4bz * q2) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
        + _4bx * q3 * (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
      double s2 =
          _2q4 * (2.0 * q2q4 - _2q1q3 - ax)
        + _2q1 * (2.0 * q1q2 + _2q3q4 - ay)
        - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
        + _4bz * q4 * (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
        + (_4bx * q3 + _4bz * q1) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
        + (_4bx * q4 - _8bz * q2) * (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
      double s3 =
        - _2q1 * (2.0 * q2q4 - _2q1q3 - ax)
        + _2q4 * (2.0 * q1q2 + _2q3q4 - ay)
        - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
        + (-_8bx * q3 - _4bz * q1) * (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
        + (_4bx * q2 + _4bz * q4) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
        + (_4bx * q1 - _8bz * q3) * (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);
      double s4 =
          _2q2 * (2.0 * q2q4 - _2q1q3 - ax)
        + _2q3 * (2.0 * q1q2 + _2q3q4 - ay)
        + (-_8bx * q4 + _4bz * q2) * (_4bx * (0.5 - q3q3 - q4q4) + _4bz * (q2q4 - q1q3) - mx)
        + (-_4bx * q1 + _4bz * q3) * (_4bx * (q2q3 - q1q4) + _4bz * (q1q2 + q3q4) - my)
        + _4bx * q2 * (_4bx * (q1q3 + q2q4) + _4bz * (0.5 - q2q2 - q3q3) - mz);

      norm = std::sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
      if (norm > 0.0) {
        norm = 1.0 / norm;  //normalise gradient step
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;
      } else {
        break;
      }

      // Convert gyro units to rad/sec
      double gx = gyro_x * kGyroScale;
      double gy = gyro_y * kGyroScale;
      double gz = gyro_z * kGyroScale;

      // Compute rate of change of quaternion
      double qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - kBeta * s1;
      double qDot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy) - kBeta * s2;
      double qDot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx) - kBeta * s3;
      double qDot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx) - kBeta * s4;

      // Integrate to yield quaternion
      q1 += qDot1 * dt;
      q2 += qDot2 * dt;
      q3 += qDot3 * dt;
      q4 += qDot4 * dt;

      norm = std::sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
      if (norm > 0.0) {
        norm = 1.0 / norm;  // normalise quaternion
        q1 = q1 * norm;
        q2 = q2 * norm;
        q3 = q3 * norm;
        q4 = q4 * norm;
      }
    } while (false);

    // Convert quaternion to angles of rotation
    double xi = -std::atan2(2*q2*q3 - 2*q1*q4, 2*(q1*q1) + 2*(q2*q2) - 1);
    double theta = -std::asin(2*q2*q4 + 2*q1*q3);
    double rho = std::atan2(2*q3*q4 - 2*q1*q2, 2*(q1*q1) + 2*(q4*q4) - 1);

    // Convert angles from radians to degrees
    xi = xi / M_PI * 180.0;
    theta = theta / M_PI * 180.0;
    rho = rho / M_PI * 180.0;

    // Adjust angles for inverted mount of MXP sensor
    theta = -theta;
    if (rho < 0)
      rho = 180 - std::abs(rho);
    else
      rho = std::abs(rho) - 180;

    // Update global state
    {
      std::lock_guard<priority_mutex> sync(m_mutex);

      m_gyro_x = gyro_x;
      m_gyro_y = gyro_y;
      m_gyro_z = gyro_z;
      m_accel_x = accel_x;
      m_accel_y = accel_y;
      m_accel_z = accel_z;
      m_mag_x = mag_x;
      m_mag_y = mag_y;
      m_mag_z = mag_z;
      m_baro = ToUShort(&m_resp[22]) * kMillibarPerLSB;
      m_temp = ToShort(&m_resp[24]) * kDegCPerLSB + kDegCOffset;

      m_accum_count += 1;
      m_accum_gyro_x += gyro_x;
      m_accum_gyro_y += gyro_y;
      m_accum_gyro_z += gyro_z;

      m_integ_gyro_x += (gyro_x - m_gyro_center_x) * dt;
      m_integ_gyro_y += (gyro_y - m_gyro_center_y) * dt;
      m_integ_gyro_z += (gyro_z - m_gyro_center_z) * dt;

      m_ahrs_q1 = q1;
      m_ahrs_q2 = q2;
      m_ahrs_q3 = q3;
      m_ahrs_q4 = q4;
      m_yaw = xi;
      m_roll = theta;
      m_pitch = rho;
    }
  }
}

/**
 * {@inheritDoc}
 */
double ADIS16448_IMU::GetAngle() const {
  return GetYaw();
}

/**
 * {@inheritDoc}
 */
double ADIS16448_IMU::GetRate() const {
  return GetRateZ();
}

double ADIS16448_IMU::GetAngleX() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_integ_gyro_x;
}

double ADIS16448_IMU::GetAngleY() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_integ_gyro_y;
}

double ADIS16448_IMU::GetAngleZ() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_integ_gyro_z;
}

double ADIS16448_IMU::GetRateX() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_gyro_x;
}

double ADIS16448_IMU::GetRateY() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_gyro_y;
}

double ADIS16448_IMU::GetRateZ() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_gyro_z;
}

double ADIS16448_IMU::GetAccelX() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_accel_x;
}

double ADIS16448_IMU::GetAccelY() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_accel_y;
}

double ADIS16448_IMU::GetAccelZ() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_accel_z;
}

double ADIS16448_IMU::GetMagX() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_mag_x;
}

double ADIS16448_IMU::GetMagY() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_mag_y;
}

double ADIS16448_IMU::GetMagZ() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_mag_z;
}

double ADIS16448_IMU::GetPitch() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_pitch;
}

double ADIS16448_IMU::GetRoll() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_roll;
}

double ADIS16448_IMU::GetYaw() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_yaw;
}

double ADIS16448_IMU::GetLastSampleTime() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_last_sample_time;
}

double ADIS16448_IMU::GetBarometricPressure() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_baro;
}

double ADIS16448_IMU::GetTemperature() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_temp;
}

double ADIS16448_IMU::GetQuaternionW() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_ahrs_q1;
}

double ADIS16448_IMU::GetQuaternionX() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_ahrs_q2;
}

double ADIS16448_IMU::GetQuaternionY() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_ahrs_q3;
}

double ADIS16448_IMU::GetQuaternionZ() const {
  std::lock_guard<priority_mutex> sync(m_mutex);
  return m_ahrs_q4;
}

/**
 * {@inheritDoc}
 */
void ADIS16448_IMU::UpdateTable() {
  auto table = GetTable();
  if (table) {
    table->PutNumber("Value", GetAngle());
    table->PutNumber("Pitch", GetPitch());
    table->PutNumber("Roll", GetRoll());
    table->PutNumber("Yaw", GetYaw());
    table->PutNumber("AccelX", GetAccelX());
    table->PutNumber("AccelY", GetAccelY());
    table->PutNumber("AccelZ", GetAccelZ());
    table->PutNumber("AngleX", GetAngleX());
    table->PutNumber("AngleY", GetAngleY());
    table->PutNumber("AngleZ", GetAngleZ());
  }
}
