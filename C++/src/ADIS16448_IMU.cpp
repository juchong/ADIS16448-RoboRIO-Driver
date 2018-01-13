/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016-2018. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ADIS16448_IMU.h"

#include <cmath>

#include <DigitalInput.h>
#include <DriverStation.h>
#include <ErrorBase.h>
#include <SmartDashboard/SendableBuilder.h>
#include <Timer.h>
#include <WPIErrors.h>

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
//static constexpr uint8_t kRegLOT_ID2 = 0x54;
//static constexpr uint8_t kRegLOT_ID1 = 0x52;
//static constexpr uint8_t kRegSERIAL_NUM = 0x58;
//static constexpr uint8_t kRegZGYRO_OFF = 0x1E;
//static constexpr uint8_t kRegYGYRO_OFF = 0x1C;
static constexpr uint8_t kRegXGYRO_OFF = 0x1A;

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

void ADIS16448_IMU::Sample::AdjustYawAxis(Axis yaw_axis) {
  switch (yaw_axis) {
    case kX: {
      // swap X and Z
      std::swap(accel_x, accel_z);
      std::swap(mag_x, mag_z);
      std::swap(gyro_x, gyro_z);
      break;
    }
    case kY: {
      // swap Y and Z
      std::swap(accel_y, accel_z);
      std::swap(mag_y, mag_z);
      std::swap(gyro_y, gyro_z);
      break;
    }
    case kZ:
    default:
      // no swap required
      break;
  }
}

/**
 * Constructor.
 */
ADIS16448_IMU::ADIS16448_IMU(Axis yaw_axis, AHRSAlgorithm algorithm)
  : m_algorithm(algorithm), m_yaw_axis(yaw_axis), m_spi(SPI::Port::kMXP) {

  // Force the IMU reset pin to toggle on startup
  //m_reset.reset(new DigitalOutput(18));  // MXP DIO8
  //m_reset->Set(false);
  //Wait(0.1);
  //m_reset->Set(true);
  //Wait(0.1);

  m_spi.SetClockRate(1000000);
  m_spi.SetMSBFirst();
  m_spi.SetSampleDataOnFalling();
  m_spi.SetClockActiveLow();
  m_spi.SetChipSelectActiveLow();

  ReadRegister(kRegPROD_ID); // dummy read

  // Validate the product ID
  if (ReadRegister(kRegPROD_ID) != 16448) {
    DriverStation::ReportError("could not find ADIS16448");
    return;
  }

  // Set IMU internal decimation to 204.8 SPS
  WriteRegister(kRegSMPL_PRD, 0x0201);

  // Enable Data Ready (LOW = Good Data) on DIO1 (PWM0 on MXP) & PoP
  WriteRegister(kRegMSC_CTRL, 0x0044);

  // Configure IMU internal Bartlett filter
  WriteRegister(kRegSENS_AVG, 0x0400);

  // Read serial number and lot ID
  //m_serial_num = ReadRegister(kRegSERIAL_NUM);
  //m_lot_id2 = ReadRegister(kRegLOT_ID2);
  //m_lot_id1 = ReadRegister(kRegLOT_ID1);

  // Configure interrupt on MXP DIO0
  m_interrupt.reset(new DigitalInput(10));
  m_interrupt->RequestInterrupts();
  m_interrupt->SetUpSourceEdge(false, true);

  // Start monitoring thread
  m_freed = false;
  m_acquire_task = std::thread(&ADIS16448_IMU::Acquire, this);
  m_calculate_task = std::thread(&ADIS16448_IMU::Calculate, this);

  if (ReadRegister(kRegXGYRO_OFF) == 0) Calibrate();

  //HALReport(HALUsageReporting::kResourceType_ADIS16448, 0);
}

/**
 * {@inheritDoc}
 */
void ADIS16448_IMU::Calibrate() {
  Wait(0.1);

  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_accum_count = 0;
    m_accum_gyro_x = 0.0;
    m_accum_gyro_y = 0.0;
    m_accum_gyro_z = 0.0;
  }

  Wait(kCalibrationSampleTime);

  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_gyro_offset_x = m_accum_gyro_x / m_accum_count;
    m_gyro_offset_y = m_accum_gyro_y / m_accum_count;
    m_gyro_offset_z = m_accum_gyro_z / m_accum_count;
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
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_integ_gyro_x = 0.0;
  m_integ_gyro_y = 0.0;
  m_integ_gyro_z = 0.0;
}

/**
 * Delete (free) the spi port used for the IMU.
 */
ADIS16448_IMU::~ADIS16448_IMU() {
  m_freed = true;
  m_samples_not_empty.notify_all();
  if (m_acquire_task.joinable()) m_acquire_task.join();
  if (m_calculate_task.joinable()) m_calculate_task.join();
}

void ADIS16448_IMU::Acquire() {
  uint8_t cmd[26], resp[26];
  cmd[0] = kGLOB_CMD;
  cmd[1] = 0;

  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_last_sample_time = Timer::GetFPGATimestamp();
  }
  while (!m_freed) {
    if (m_interrupt->WaitForInterrupt(kTimeout) ==
        InterruptableSensorBase::WaitResult::kTimeout)
      continue;

    double sample_time = m_interrupt->ReadFallingTimestamp();
    double dt;
    {
      std::lock_guard<wpi::mutex> sync(m_mutex);
      dt = sample_time - m_last_sample_time;
      m_last_sample_time = sample_time;
    }

    m_spi.Transaction(cmd, resp, 26);

    double gyro_x = ToShort(&resp[4]) * kDegreePerSecondPerLSB;
    double gyro_y = ToShort(&resp[6]) * kDegreePerSecondPerLSB;
    double gyro_z = ToShort(&resp[8]) * kDegreePerSecondPerLSB;
    double accel_x = ToShort(&resp[10]) * kGPerLSB;
    double accel_y = ToShort(&resp[12]) * kGPerLSB;
    double accel_z = ToShort(&resp[14]) * kGPerLSB;
    double mag_x = ToShort(&resp[16]) * kMilligaussPerLSB;
    double mag_y = ToShort(&resp[18]) * kMilligaussPerLSB;
    double mag_z = ToShort(&resp[20]) * kMilligaussPerLSB;
    double baro = ToUShort(&resp[22]) * kMillibarPerLSB;
    double temp = ToShort(&resp[24]) * kDegCPerLSB + kDegCOffset;

    {
      std::lock_guard<wpi::mutex> sync(m_samples_mutex);
      // If the FIFO is full, just drop it
      if (m_calculate_started && m_samples_count < kSamplesDepth) {
        Sample& sample = m_samples[m_samples_put_index];
        sample.gyro_x = gyro_x;
        sample.gyro_y = gyro_y;
        sample.gyro_z = gyro_z;
        sample.accel_x = accel_x;
        sample.accel_y = accel_y;
        sample.accel_z = accel_z;
        sample.mag_x = mag_x;
        sample.mag_y = mag_y;
        sample.mag_z = mag_z;
        sample.baro = baro;
        sample.temp = temp;
        sample.dt = dt;
        ++m_samples_put_index;
        if (m_samples_put_index == (kSamplesDepth + 2))
          m_samples_put_index = 0;
        ++m_samples_count;
        m_samples_not_empty.notify_all();
      }
    }

    // Update global state
    {
      std::lock_guard<wpi::mutex> sync(m_mutex);
      m_gyro_x = gyro_x;
      m_gyro_y = gyro_y;
      m_gyro_z = gyro_z;
      m_accel_x = accel_x;
      m_accel_y = accel_y;
      m_accel_z = accel_z;
      m_mag_x = mag_x;
      m_mag_y = mag_y;
      m_mag_z = mag_z;
      m_baro = baro;
      m_temp = temp;

      ++m_accum_count;
      m_accum_gyro_x += gyro_x;
      m_accum_gyro_y += gyro_y;
      m_accum_gyro_z += gyro_z;

      m_integ_gyro_x += (gyro_x - m_gyro_offset_x) * dt;
      m_integ_gyro_y += (gyro_y - m_gyro_offset_y) * dt;
      m_integ_gyro_z += (gyro_z - m_gyro_offset_z) * dt;
    }
  }
}

void ADIS16448_IMU::Calculate() {
  while (!m_freed) {
    // Wait for next sample and get it
    Sample* sample;
    {
      std::unique_lock<wpi::mutex> sync(m_samples_mutex);
      m_calculate_started = true;
      m_samples_not_empty.wait(sync,
                               [=] { return m_samples_count != 0 || m_freed; });
      if (m_freed) return;
      sample = &m_samples[m_samples_take_index];
      ++m_samples_take_index;
      if (m_samples_take_index == (kSamplesDepth + 2))
        m_samples_take_index = 0;
      --m_samples_count;
    }

    switch (m_algorithm) {
      case kMadgwick:
        CalculateMadgwick(*sample, 0.4);
        break;
      case kComplementary:
      default:
        CalculateComplementary(*sample);
        break;
    }
  }
}

void ADIS16448_IMU::CalculateMadgwick(Sample& sample, double beta) {
  // Make local copy of quaternion and angle global state
  double q1, q2, q3, q4;
  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    q1 = m_ahrs_q1;
    q2 = m_ahrs_q2;
    q3 = m_ahrs_q3;
    q4 = m_ahrs_q4;
  }

  // Swap axis as appropriate for yaw axis selection
  sample.AdjustYawAxis(m_yaw_axis);

  // Kalman calculation
  // Code originated from: https://decibel.ni.com/content/docs/DOC-18964
  do {
    // If true, only use gyros and magnetos for updating the filter.
    bool excludeAccel = false;

    // Convert accelerometer units to m/sec/sec
    double ax = sample.accel_x * kAccelScale;
    double ay = sample.accel_y * kAccelScale;
    double az = sample.accel_z * kAccelScale;
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
    double mx = sample.mag_x * kMagScale;
    double my = sample.mag_y * kMagScale;
    double mz = sample.mag_z * kMagScale;
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
    double gx = sample.gyro_x * kGyroScale;
    double gy = sample.gyro_y * kGyroScale;
    double gz = sample.gyro_z * kGyroScale;

    // Compute rate of change of quaternion
    double qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - kBeta * s1;
    double qDot2 = 0.5 * ( q1 * gx + q3 * gz - q4 * gy) - kBeta * s2;
    double qDot3 = 0.5 * ( q1 * gy - q2 * gz + q4 * gx) - kBeta * s3;
    double qDot4 = 0.5 * ( q1 * gz + q2 * gy - q3 * gx) - kBeta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * sample.dt;
    q2 += qDot2 * sample.dt;
    q3 += qDot3 * sample.dt;
    q4 += qDot4 * sample.dt;

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
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_ahrs_q1 = q1;
    m_ahrs_q2 = q2;
    m_ahrs_q3 = q3;
    m_ahrs_q4 = q4;
    m_yaw = xi;
    m_roll = theta;
    m_pitch = rho;
  }
}

// Thank you to the RoboBees for providing this elegant AHRS implementation
// to the FIRST community!
void ADIS16448_IMU::CalculateComplementary(Sample& sample) {
  // Description:
  // Accepts calibrated Rate Gyro, Accelerometer, and Magnetometer sensor
  // readings and applies a Complementary Filter to fuse them into a single
  // composite sensor which provides accurate and stable rotation indications
  // (Pitch, Roll, and Yaw).  This sensor fusion approach effectively
  // combines the individual sensor's best respective properties while
  // mitigating their shortfalls.
  // 
  // Design:
  // The Complementary Filter is an algorithm that allows a pair of sensors
  // to contribute differently to a common, composite measurement result.
  // It effectively applies a low pass filter to one sensor, and a high pass
  // filter to the other, then proportionally recombines them in such a way
  // to maintain the original unit of measurement.  It is computationally
  // inexpensive when compared to alternative estimation techniques such as
  // the Kalman filter.  The algorithm is given by:
  // 
  // angle(n) = (alpha)*(angle(n-1) + gyrorate * dt) + (1-alpha)*(accel or mag);
  // 
  // where : 
  // 
  // alpha = tau / (tau + dt)
  // 
  // This implementation uses the average Gyro rate across the dt period, so
  // above gyrorate = [(gyrorate(n)-gyrorate(n-1)]/2
  // 
  // Essentially, for Pitch and Roll, the slow moving (lower frequency) part
  // of the rotation estimate is taken from the Accelerometer - ignoring the
  // high noise level, and the faster moving (higher frequency) part is taken
  // from the Rate Gyro - ignoring the slow Gyro drift.  Same for Yaw, except
  // that the Magnetometer replaces the Accelerometer to source the slower
  // moving component.  This is because Pitch and Roll can be referenced to
  // the Accelerometer's sense of the Earth's gravity vector.  Yaw cannot be
  // referenced to this vector since this rotation does not cause any
  // relative angular change, but it can be referenced to magnetic North.
  // The parameter 'tau' is the time constant that defines the boundary
  // between the low and high pass filters.  Both tau and the sample time,
  // dt, affect the parameter 'alpha', which sets the balance point for how
  // much of which sensor is 'trusted' to contribute to the rotation estimate.
  // 
  // The Complementary Filter algorithm is applied to each X/Y/Z rotation
  // axis to compute R/P/Y outputs, respectively.
  // 
  // Magnetometer readings are tilt-compensated when Tilt-Comp-(Yaw) is
  // asserted (True), by the IMU TILT subVI.  This creates what is known as a
  // tilt-compensated compass, which allows Yaw to be insensitive to the
  // effects of a non-level sensor, but generates error in Yaw during
  // movement (coordinate acceleration).
  // 
  // The Yaw "South" crossing detector is necessary to allow a smooth
  // transition across the +/- 180 deg discontinuity (inherent in the ATAN
  // function).  Since -180 deg is congruent with +180 deg, Yaw needs to jump
  // between these values when crossing South (North is 0 deg).  The design
  // depends upon comparison of successive Yaw readings to detect a
  // cross-over event.  The cross-over detector monitors the current reading
  // and evaluates how far it is from the previous reading.  If it is greater
  // than the previous reading by the Discriminant (= 180 deg), then Yaw just
  // crossed South.
  // 
  // By choosing 180 as the Discriminant, the only way the detector can
  // produce a false positive, assuming a loop iteration of 70 msec, is for
  // it to rotate >2,571 dps ... (2,571=180/.07).  This is faster than the ST
  // L3GD20 Gyro can register.  The detector produces a Boolean True upon
  // detecting a South crossing.  This is used to alter the (n-1) Yaw which
  // was previously stored, either adding or subtracting 360 degrees as
  // required to place the previous Yaw in the correct quadrant whenever
  // crossing occurs.  The Modulus function cannot be used here as the
  // Complementary Filter algorithm has 'state' (needs to remember previous
  // Yaw).
  // 
  // We are in effect stitching together two ends of a ruler for 'modular
  // arithmetic' (clock math).
  // 
  // Inputs:
  // GYRO - Gyro rate and sample time measurements.
  // ACCEL - Acceleration measurements.
  // MAG - Magnetic measurements.
  // TAU ACC - tau parameter used to set sensor balance between Accel and
  //           Gyro for Roll and Pitch.
  // TAU MAG - tau parameter used to set sensor balance between Mag and Gyro
  //           for Yaw.
  // TILT COMP (Yaw) - Enables Yaw tilt-compensation if True.
  // 
  // Outputs:
  // ROLL - Filtered Roll about sensor X-axis.
  // PITCH - Filtered Pitch about sensor Y-axis.
  // YAW - Filtered Yaw about sensor Z-axis.
  // 
  // Implementation:
  // It's best to establish the optimum loop sample time first.  See IMU READ
  // implementation notes for guidance.  Each tau parameter should then be
  // adjusted to achieve optimum sensor fusion.  tau acc affects Roll and
  // Pitch, tau mag affects Yaw.  Start at value 1 or 2 and decrease by half
  // each time until the result doesn't drift, but not so far that the result
  // gets noisy.  An optimum tau for this IMU is likely in the range of 1.0
  // to 0.01, for a loop sample time between 10 and 100 ms.
  // 
  // Note that both sample timing (dt) and tau both affect the balance
  // parameter, 'alpha'.  Adjusting either dt or tau will require the other
  // to be readjusted to maintain a particular filter performance.
  // 
  // It is likely best to set Yaw tilt-compensation to off (False) if the Yaw
  // value is to be used as feedback in a closed loop control application.
  // The tradeoff is that Yaw will only be accurate while the robot is level.
  // 
  // Since a Yaw of -180 degrees is congruent with +180 degrees (they
  // represent the same direction), it is possible that the Yaw output will
  // oscillate between these two values when the sensor happens to be
  // pointing due South, as sensor noise causes slight variation.  You will
  // need to account for this possibility if you are using the Yaw value for
  // decision-making in code.
  // 
  // ----- The RoboBees FRC Team 836! -----
  // Complement your passion to solve problems with a STEM Education!

  // Compensate for PCB-Up Mounting Config.
  sample.gyro_y = -sample.gyro_y;
  sample.gyro_z = -sample.gyro_z;
  sample.accel_y = -sample.accel_y;
  sample.accel_z = -sample.accel_z;
  sample.mag_y = -sample.mag_y;
  sample.mag_z = -sample.mag_z;

  // Swap axis as appropriate for yaw axis selection
  sample.AdjustYawAxis(m_yaw_axis);

  static constexpr double tau_acc = 0.95;
  static constexpr double tau_mag = 0.04;

  double roll, pitch, yaw;
  bool tilt_comp_yaw;
  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    roll = m_roll;
    pitch = m_pitch;
    yaw = m_yaw;
    tilt_comp_yaw = m_tilt_comp_yaw;
  }

  // Calculate mag angle in degrees
  double mag_angle = std::atan2(sample.mag_y, sample.mag_x) / M_PI * 180.0;

  // Tilt compensation:
  // see http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
  // for derivation of Pitch and Roll equations.  Used eqs 37 & 38 as Rxyz.
  // Eqs 42 & 43, as Ryxz, produce same values within Pitch & Roll
  // constraints.
  // 
  // Freescale's Pitch/Roll derivation is preferred over ST's as it does not
  // degrade due to the Sine function linearity assumption.
  // 
  // Pitch is accurate over +/- 90 degree range, and Roll is accurate within
  // +/- 180 degree range - as long as accelerometer is only sensing
  // acceleration due to gravity.  Movement (coordinate acceleration) will
  // add error to Pitch and Roll indications.
  // 
  // Yaw is not obtainable from an accelerometer due to its geometric 
  // relationship with the Earth's gravity vector.  (Would have same problem
  // on Mars.)
  // 
  // see http://www.pololu.com/file/0J434/LSM303DLH-compass-app-note.pdf
  // for derivation of Yaw equation.  Used eq 12 in Appendix A (eq 13 is
  // replaced by ATAN2 function).  Yaw is obtainable from the magnetometer,
  // but is sensitive to any tilt from horizontal.  This uses Pitch and Roll
  // values from above for tilt compensation of Yaw, resulting in a
  // tilt-compensated compass.
  // 
  // As with Pitch/Roll, movement (coordinate acceleration) will add error to
  // Yaw indication.

  // Accel
  double tilt_pitch_rad = std::atan2(-sample.accel_x, std::sqrt(sample.accel_y * sample.accel_y + sample.accel_z * sample.accel_z));
  double tilt_pitch = tilt_pitch_rad / M_PI * 180.0;

  int accel_z_signum;
  if (sample.accel_z < 0)
    accel_z_signum = -1;
  else if (sample.accel_z > 0)
    accel_z_signum = 1;
  else
    accel_z_signum = 0;
  double tilt_roll_rad = std::atan2(sample.accel_y, std::sqrt(sample.accel_x * sample.accel_x * 0.01 + sample.accel_z * sample.accel_z) * accel_z_signum);
  double tilt_roll = tilt_roll_rad / M_PI * 180.0;

  // Mag
  double tilt_yaw;
  if (tilt_comp_yaw) {
    double sin_pitch = std::sin(tilt_pitch_rad);
    double cos_pitch = std::cos(tilt_pitch_rad);
    double sin_roll = std::sin(tilt_roll_rad);
    double cos_roll = std::cos(tilt_roll_rad);
    double mx2 = sample.mag_x * cos_pitch + sample.mag_z * sin_pitch;
    double my2 = sample.mag_x * sin_roll * sin_pitch + sample.mag_y * cos_roll - sample.mag_z * sin_roll * cos_pitch;
    //double mz2 = -sample.mag_x * cos_roll * sin_pitch + sample.mag_y * sin_roll + sample.mag_z * cos_roll * cos_pitch;
    tilt_yaw = std::atan2(my2, mx2) / M_PI * 180.0;
  } else {
    tilt_yaw = mag_angle;
  }

  // Positive rotation of Magnetometer is clockwise when looking in + Z
  // direction.  This is subtracted from 0 deg to reverse rotation
  // direction, as it needs to be aligned with the definition of positive
  // Gyroscope rotation, (which is CCW looking in + Z direction), to enable
  // sensor fusion.
  //
  // 0 degrees is due magnetic North.
  tilt_yaw = -tilt_yaw;

  // "South" crossing Detector
  if (std::abs(mag_angle - m_mag_angle_prev) >= 180) {
    if (m_mag_angle_prev < 0) {
      yaw += -360;
    } else if (m_mag_angle_prev > 0) {
      yaw += 360;
    }
  }
  m_mag_angle_prev = mag_angle;

  // alpha = tau / (tau + dt)
  double alpha_acc = tau_acc / (tau_acc + sample.dt);
  double alpha_mag = tau_mag / (tau_mag + sample.dt);

  // gyrorate = [(gyrorate(n)-gyrorate(n-1)]/2
  // angle(n) = (alpha)*(angle(n-1) + gyrorate * dt) + (1-alpha)*(accel or mag);
  if (m_first) {
    m_gyro_x_prev = sample.gyro_x;
    m_gyro_y_prev = sample.gyro_y;
    m_gyro_z_prev = sample.gyro_z;
    m_first = false;
  }
  roll =
      alpha_acc * (roll + sample.dt * (sample.gyro_x + m_gyro_x_prev) / 2.0) +
      (1 - alpha_acc) * tilt_roll;
  pitch =
      alpha_acc * (pitch + sample.dt * (sample.gyro_y + m_gyro_y_prev) / 2.0) +
      (1 - alpha_acc) * tilt_pitch;
  yaw =
      alpha_mag * (yaw + sample.dt * (sample.gyro_z + m_gyro_z_prev) / 2.0) +
      (1 - alpha_mag) * tilt_yaw;
  m_gyro_x_prev = sample.gyro_x;
  m_gyro_y_prev = sample.gyro_y;
  m_gyro_z_prev = sample.gyro_z;

  // Update global state
  {
    std::lock_guard<wpi::mutex> sync(m_mutex);
    m_roll = roll;
    m_pitch = pitch;
    m_yaw = yaw;
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
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_gyro_x;
}

double ADIS16448_IMU::GetAngleY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_gyro_y;
}

double ADIS16448_IMU::GetAngleZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_integ_gyro_z;
}

double ADIS16448_IMU::GetRateX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_x;
}

double ADIS16448_IMU::GetRateY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_y;
}

double ADIS16448_IMU::GetRateZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_gyro_z;
}

double ADIS16448_IMU::GetAccelX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_x;
}

double ADIS16448_IMU::GetAccelY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_y;
}

double ADIS16448_IMU::GetAccelZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_accel_z;
}

double ADIS16448_IMU::GetMagX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_mag_x;
}

double ADIS16448_IMU::GetMagY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_mag_y;
}

double ADIS16448_IMU::GetMagZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_mag_z;
}

double ADIS16448_IMU::GetPitch() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_pitch;
}

double ADIS16448_IMU::GetRoll() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_roll;
}

double ADIS16448_IMU::GetYaw() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_yaw;
}

double ADIS16448_IMU::GetLastSampleTime() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_last_sample_time;
}

double ADIS16448_IMU::GetBarometricPressure() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_baro;
}

double ADIS16448_IMU::GetTemperature() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_temp;
}

double ADIS16448_IMU::GetQuaternionW() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_ahrs_q1;
}

double ADIS16448_IMU::GetQuaternionX() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_ahrs_q2;
}

double ADIS16448_IMU::GetQuaternionY() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_ahrs_q3;
}

double ADIS16448_IMU::GetQuaternionZ() const {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  return m_ahrs_q4;
}

// Enable or disable yaw tilt-compensation for the Complementary AHRS.
// Has no effect on the Kalman AHRS.
//
// It is likely best to set Yaw tilt-compensation to off (False) if the Yaw
// value is to be used as feedback in a closed loop control application.
// The tradeoff is that Yaw will only be accurate while the robot is level.
void ADIS16448_IMU::SetTiltCompYaw(bool enabled) {
  std::lock_guard<wpi::mutex> sync(m_mutex);
  m_tilt_comp_yaw = enabled;
}

void ADIS16448_IMU::InitSendable(SendableBuilder& builder) {
  builder.SetSmartDashboardType("ADIS16448 IMU");
  auto value = builder.GetEntry("Value").GetHandle();
  auto pitch = builder.GetEntry("Pitch").GetHandle();
  auto roll = builder.GetEntry("Roll").GetHandle();
  auto yaw = builder.GetEntry("Yaw").GetHandle();
  auto accelX = builder.GetEntry("AccelX").GetHandle();
  auto accelY = builder.GetEntry("AccelY").GetHandle();
  auto accelZ = builder.GetEntry("AccelZ").GetHandle();
  auto angleX = builder.GetEntry("AngleX").GetHandle();
  auto angleY = builder.GetEntry("AngleY").GetHandle();
  auto angleZ = builder.GetEntry("AngleZ").GetHandle();
  builder.SetUpdateTable([=]() {
	nt::NetworkTableEntry(value).SetDouble(GetAngle());
	nt::NetworkTableEntry(pitch).SetDouble(GetPitch());
	nt::NetworkTableEntry(roll).SetDouble(GetRoll());
	nt::NetworkTableEntry(yaw).SetDouble(GetYaw());
	nt::NetworkTableEntry(accelX).SetDouble(GetAccelX());
	nt::NetworkTableEntry(accelY).SetDouble(GetAccelY());
	nt::NetworkTableEntry(accelZ).SetDouble(GetAccelZ());
	nt::NetworkTableEntry(angleX).SetDouble(GetAngleX());
	nt::NetworkTableEntry(angleY).SetDouble(GetAngleY());
	nt::NetworkTableEntry(angleZ).SetDouble(GetAngleZ());
  });
}
