/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*                                                                            */
/* Modified by Juan Chong - juan.chong@analog.com                             */
/*----------------------------------------------------------------------------*/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>

#include <frc/DigitalOutput.h>
#include <frc/DigitalSource.h>
#include <frc/DigitalInput.h>
#include <frc/GyroBase.h>
#include <frc/SPI.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <wpi/mutex.h>
#include <wpi/condition_variable.h>

namespace frc {

/**
 * Use DMA SPI to read rate, acceleration, and magnetometer data from the ADIS16448 IMU
 * and return the robots heading relative to a starting position, AHRS, and instant measurements
 *
 * The ADIS16448 gyro angle outputs track the robot's heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of rotation returned by 
 * the IMU. When the class is instantiated, a short calibration routine is performed where the 
 * IMU samples the gyros while at rest to determine the initial offset. This is subtracted from 
 * each sample to determine the heading.
 *
 * This class is for the ADIS16448 IMU connected via the SPI port available on the RoboRIO MXP port.
 */

class ADIS16448_IMU : public GyroBase {
  public:

    enum AHRSAlgorithm { kComplementary, kMadgwick };
    enum IMUAxis { kX, kY, kZ };

    /**
    * IMU constructor on onboard MXP CS0, Z-up orientation, and complementary AHRS computation.
    */
    ADIS16448_IMU();

    /**
     * IMU constructor on the specified MXP port and orientation.
     * 
     * @param yaw_axis The axis where gravity is present. Valid options are kX, kY, and kZ
     * @param algorithm The AHRS algorithm to use. Valid options are kComplementary and kMadgwick
     * @param port The SPI port where the IMU is connected.
     */
    explicit ADIS16448_IMU(IMUAxis yaw_axis, AHRSAlgorithm algorithm, SPI::Port port);

    ~ADIS16448_IMU();

    ADIS16448_IMU(ADIS16448_IMU&&) = default;
    ADIS16448_IMU& operator=(ADIS16448_IMU&&) = default;
    
    /**
     * Initialize the IMU.
     *
     * Perform gyro offset calibration by collecting data for a number of seconds and 
     * computing the center value. The center value is subtracted from subsequent
     * measurements. 
     *
     * It's important to make sure that the robot is not moving while the
     * centering calculations are in progress, this is typically done when the
     * robot is first turned on while it's sitting at rest before the match
     * starts.
     * 
     * The calibration routine can be triggered by the user during runtime.
     */
    void Calibrate() override;

    /**
     * Reset the gyro.
     *
     * Resets the gyro accumulations to a heading of zero. This can be used if 
     * there is significant drift in the gyro and it needs to be recalibrated
     * after running.
     */
    void Reset() override;

    /**
     * Return the actual angle in degrees that the robot is currently facing.
     *
     * The angle is based on the current accumulator value corrected by
     * offset calibration and built-in IMU calibration. The angle is continuous, 
     * that is it will continue from 360->361 degrees. This allows algorithms 
     * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
     * from 360 to 0 on the second time around. The axis returned by this 
     * function is adjusted fased on the configured yaw_axis.
     *
     * @return the current heading of the robot in degrees. This heading is based
     *         on integration of the returned rate from the gyro.
     */
    double GetAngle() const override;

    /**
     * Return the rate of rotation of the yaw_axis gyro.
     *
     * The rate is based on the most recent reading of the gyro value
     *
     * @return the current rate in degrees per second
     */
    double GetRate() const override;
    
    /**
     * Return the IMU X-axis integrated angle in degrees.
     *
     * The angle is based on the current accumulator value corrected by
     * offset calibration and built-in IMU calibration. The angle is continuous, 
     * that is it will continue from 360->361 degrees. This allows algorithms 
     * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
     * from 360 to 0 on the second time around. 
     *
     * @return the current accumulated value of the X-axis in degrees. 
     */
    double GetAngleX() const;

    /**
     * Return the IMU Y-axis integrated angle in degrees.
     *
     * The angle is based on the current accumulator value corrected by
     * offset calibration and built-in IMU calibration. The angle is continuous, 
     * that is it will continue from 360->361 degrees. This allows algorithms 
     * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
     * from 360 to 0 on the second time around. 
     *
     * @return the current accumulated value of the Y-axis in degrees. 
     */
    double GetAngleY() const;

    /**
     * Return the IMU Z-axis integrated angle in degrees.
     *
     * The angle is based on the current accumulator value corrected by
     * offset calibration and built-in IMU calibration. The angle is continuous, 
     * that is it will continue from 360->361 degrees. This allows algorithms 
     * that wouldn't want to see a discontinuity in the gyro output as it sweeps 
     * from 360 to 0 on the second time around. 
     *
     * @return the current accumulated value of the Z-axis in degrees. 
     */
    double GetAngleZ() const;

    /**
     * Return the rate of rotation of the X-axis gyro.
     *
     * The rate is based on the most recent reading of the gyro value
     *
     * @return the current rate of the X-axis gyro in degrees per second
     */
    double GetRateX() const;

    /**
     * Return the rate of rotation of the Y-axis gyro.
     *
     * The rate is based on the most recent reading of the gyro value
     *
     * @return the current rate of the Y-axis gyro in degrees per second
     */
    double GetRateY() const;

    /**
     * Return the rate of rotation of the Z-axis gyro.
     *
     * The rate is based on the most recent reading of the gyro value
     *
     * @return the current rate of the Z-axis gyro in degrees per second
     */
    double GetRateZ() const;

    /**
     * Return acceleration of the X-axis accelerometer.
     *
     * The acceleration measurement is based on the most recent reading of 
     * the accelerometer value.
     *
     * @return the current acceleration of the X-axis accelerometer in g's
     */
    double GetAccelX() const;

    /**
     * Return acceleration of the Y-axis accelerometer.
     *
     * The acceleration measurement is based on the most recent reading of 
     * the accelerometer value.
     *
     * @return the current acceleration of the Y-axis accelerometer in g's
     */
    double GetAccelY() const;

    /**
     * Return acceleration of the Z-axis accelerometer.
     *
     * The acceleration measurement is based on the most recent reading of 
     * the accelerometer value.
     *
     * @return the current acceleration of the Z-axis accelerometer in g's
     */
    double GetAccelZ() const;

    /**
     * Return magnetic field intensity of the X-axis magnetometer.
     *
     * The megnetic field intensity measurement is based on the most recent reading of 
     * the magnetometer value.
     *
     * @return the current magnetic field intensity of the X-axis magnetometer in mgauss
     */
    double GetMagX() const;

    /**
     * Return magnetic field intensity of the Y-axis magnetometer.
     *
     * The megnetic field intensity measurement is based on the most recent reading of 
     * the magnetometer value.
     *
     * @return the current magnetic field intensity of the Y-axis magnetometer in mgauss
     */
    double GetMagY() const;

    /**
     * Return magnetic field intensity of the Z-axis magnetometer.
     *
     * The megnetic field intensity measurement is based on the most recent reading of 
     * the magnetometer value.
     *
     * @return the current magnetic field intensity of the Z-axis magnetometer in mgauss
     */
    double GetMagZ() const;

    /**
     * Return the pitch angle measurement with reference to the yaw_axis configuration.
     *
     * The pitch angle measurement is based on a combination of gyroscope, accelerometer,
     * and magnetometer measurements fused using the configured AHRS algorithm. Note that the 
     * angle accumulation WILL roll over.
     *
     * @return the pitch angle measurement with respect to the yaw_axis configuration in degrees
     */
    double GetPitch() const;

    /**
     * Return the roll angle measurement with reference to the yaw_axis configuration.
     *
     * The roll angle measurement is based on a combination of gyroscope, accelerometer,
     * and magnetometer measurements fused using the configured AHRS algorithm. Note that the 
     * angle accumulation WILL roll over.
     *
     * @return the roll angle measurement with respect to the yaw_axis configuration in degrees
     */
    double GetRoll() const;

    /**
     * Return the yaw angle measurement with reference to the yaw_axis configuration.
     *
     * The yaw angle measurement is based on a combination of gyroscope, accelerometer,
     * and magnetometer measurements fused using the configured AHRS algorithm. Note that the 
     * angle accumulation WILL roll over.
     *
     * @return the yaw angle measurement with respect to the yaw_axis configuration in degrees
     */
    double GetYaw() const;

    /**
     * Return the delta-time calculation.
     *
     * The delta-time value is calculated by subtracting the previous data timestamp from the
     * current timestamp. 
     *
     * @return the delta-time calculation of each data capture in seconds
     */
    double Getdt() const;

    /**
     * Return IMU status.
     *
     * IMU status can be deciphered by referring to the DIAG_STAT (Table 31) in the ADIS16448 datasheet.
     * 
     * @return the status code read from the IMU
     */
    double GetStatus() const;

    /**
     * Return barometric pressure as measured by the IMU.
     *
     * The barometric pressure measurement is based on the most recent reading of the barometer.
     * Note that the barometer is updated slower than the rest of the sensors (51.2 sps)
     * 
     * @return the current barometric pressure in mbar
     */
    double GetBarometricPressure() const;

    /**
     * Return temperature as measured by the IMU.
     *
     * The temperature measurement is based on the most recent reading of the temperature sensor
     * built in to the inertial sensors. 
     * 
     * @return the current temperature in degrees C
     */
    double GetTemperature() const;

    /**
     * Return quaternions used for AHRS calculations.
     * 
     * @return the quaternion used in AHRS calculations
     */
    double GetQuaternionW() const;
    double GetQuaternionX() const;
    double GetQuaternionY() const;
    double GetQuaternionZ() const;

    /**
     * Enable/Disable yaw tilt compensation for the Complementary AHRS.
     * 
     * It is likely best to set yaw tilt compensation to false if the yaw
     * value is to be used as feedback in a closed-loop control application. 
     * The tradeoff is that yaw will only be accurate while the robot is level.
     * 
     * Note that this setting has no effect when using the Madgwick AHRS calculation method.
     */
    void SetTiltCompYaw(bool enabled);

    void InitSendable(SendableBuilder& builder) override;

private:
  // Sample from the IMU
  struct Sample {
    double gyro_x;
    double gyro_y;
    double gyro_z;
    double accel_x;
    double accel_y;
    double accel_z;
    double mag_x;
    double mag_y;
    double mag_z;
    double baro;
    double temp;
    double status;
    double dt;

    // Swap axis as appropriate for yaw axis selection
    void AdjustYawAxis(IMUAxis yaw_axis);
  };

  uint16_t ReadRegister(uint8_t reg);
  void WriteRegister(uint8_t reg, uint16_t val);
  void Acquire();
  void Calculate();
  void CalculateMadgwick(Sample& sample, double beta);
  void CalculateComplementary(Sample& sample);

  // AHRS algorithm
  AHRSAlgorithm m_algorithm;

  // AHRS yaw axis
  IMUAxis m_yaw_axis;

  // Gyro offset
  double m_gyro_offset_x = 0.0;
  double m_gyro_offset_y = 0.0;
  double m_gyro_offset_z = 0.0;

  // Last read values (post-scaling)
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
  double m_status = 0.0;
  double m_dt = 0.0;

  // Accumulated gyro values (for offset calculation)
  int m_accum_count = 0;
  double m_accum_gyro_x = 0.0;
  double m_accum_gyro_y = 0.0;
  double m_accum_gyro_z = 0.0;

  // Integrated gyro values
  double m_integ_gyro_x = 0.0;
  double m_integ_gyro_y = 0.0;
  double m_integ_gyro_z = 0.0;

  // Last sample time
  double m_last_sample_time = 0.0;
  double timestamp_old = 0;

  // Kalman (AHRS)
  double m_ahrs_q1 = 1, m_ahrs_q2 = 0, m_ahrs_q3 = 0, m_ahrs_q4 = 0;

  // Complementary AHRS
  bool m_first = true;
  double m_gyro_x_prev;
  double m_gyro_y_prev;
  double m_gyro_z_prev;
  double m_mag_angle_prev = 0.0;
  bool m_tilt_comp_yaw = true;

  // AHRS outputs
  double m_yaw = 0.0;
  double m_roll = 0.0;
  double m_pitch = 0.0;

  std::atomic_bool m_freed;

  SPI m_spi;
  std::unique_ptr<frc::DigitalOutput> m_reset_out;
  std::unique_ptr<frc::DigitalInput> m_reset_in;
  std::unique_ptr<frc::DigitalSource> m_interrupt;

  std::thread m_acquire_task;
  std::thread m_calculate_task;

  mutable wpi::mutex m_mutex;

  // Samples FIFO.  We make the FIFO 2 longer than it needs
  // to be so the input and output never overlap (we hold a reference
  // to the output while the lock is released).
  static constexpr int kSamplesDepth = 10;
  Sample m_samples[kSamplesDepth + 2];
  wpi::mutex m_samples_mutex;
  wpi::condition_variable m_samples_not_empty;
  int m_samples_count = 0;
  int m_samples_take_index = 0;
  int m_samples_put_index = 0;
  bool m_calculate_started = false;
};

} //namespace frc