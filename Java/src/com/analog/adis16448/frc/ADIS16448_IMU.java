/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.analog.adis16448.frc;

import java.nio.ByteOrder;
import java.nio.ByteBuffer;
import java.util.concurrent.atomic.AtomicBoolean;

//import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tInstances;
//import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
//import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.InterruptableSensorBase;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is for the ADIS16448 IMU that connects to the RoboRIO MXP port.
 */
public class ADIS16448_IMU extends GyroBase implements Gyro, PIDSource, LiveWindowSendable {
  private static final double kTimeout = 0.1;
  private static final double kCalibrationSampleTime = 5.0;
  private static final double kDegreePerSecondPerLSB = 1.0/25.0;
  private static final double kGPerLSB = 1.0/1200.0;
  private static final double kMilligaussPerLSB = 1.0/7.0;
  private static final double kMillibarPerLSB = 0.02;
  private static final double kDegCPerLSB = 0.07386;
  private static final double kDegCOffset = 31;

  private static final int kGLOB_CMD = 0x3E;
  private static final int kRegSMPL_PRD = 0x36;
  private static final int kRegSENS_AVG = 0x38;
  private static final int kRegMSC_CTRL = 0x34;
  private static final int kRegPROD_ID = 0x56;

  // gyro center
  private double m_gyro_center_x = 0.0;
  private double m_gyro_center_y = 0.0;
  private double m_gyro_center_z = 0.0;

  // last read values (post-scaling)
  private double m_gyro_x = 0.0;
  private double m_gyro_y = 0.0;
  private double m_gyro_z = 0.0;
  private double m_accel_x = 0.0;
  private double m_accel_y = 0.0;
  private double m_accel_z = 0.0;
  private double m_mag_x = 0.0;
  private double m_mag_y = 0.0;
  private double m_mag_z = 0.0;
  private double m_baro = 0.0;
  private double m_temp = 0.0;

  // accumulated gyro values (for offset calculation)
  private int m_accum_count = 0;
  private double m_accum_gyro_x = 0.0;
  private double m_accum_gyro_y = 0.0;
  private double m_accum_gyro_z = 0.0;

  // integrated gyro values
  private double m_integ_gyro_x = 0.0;
  private double m_integ_gyro_y = 0.0;
  private double m_integ_gyro_z = 0.0;

  // last sample time
  private double m_last_sample_time = 0.0;

  // Kalman (AHRS)
  private static final double kGyroScale = 0.0174533;   // rad/sec
  private static final double kAccelScale = 9.80665;    // mg/sec/sec
  private static final double kMagScale = 0.1;          // uTesla
  private static final double kBeta = 1;
  private double m_ahrs_q1 = 1, m_ahrs_q2 = 0, m_ahrs_q3 = 0, m_ahrs_q4 = 0;
  private double m_yaw = 0.0;
  private double m_roll = 0.0;
  private double m_pitch = 0.0;

  private AtomicBoolean m_freed = new AtomicBoolean(false);

  private SPI m_spi;
  private ByteBuffer m_cmd, m_resp;
  private DigitalInput m_interrupt;
  private static class ReadTask implements Runnable {
    private ADIS16448_IMU imu;
    public ReadTask(ADIS16448_IMU imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.calculate();
    }
  }
  private Thread m_task;

  /**
   * Constructor.
   */
  public ADIS16448_IMU() {
    m_spi = new SPI(SPI.Port.kMXP);
    m_spi.setClockRate(1000000);
    m_spi.setMSBFirst();
    m_spi.setSampleDataOnFalling();
    m_spi.setClockActiveLow();
    m_spi.setChipSelectActiveLow();

    readRegister(kRegPROD_ID); // dummy read
    
    // Validate the part ID
    if (readRegister(kRegPROD_ID) != 16448) {
      m_spi.free();
      m_spi = null;
      DriverStation.reportError("could not find ADIS16448", false);
      return;
    }

    // Set IMU internal decimation to 102.4 SPS
    writeRegister(kRegSMPL_PRD, 769);

    // Enable Data Ready (LOW = Good Data) on DIO1 (PWM0 on MXP)
    writeRegister(kRegMSC_CTRL, 4);

    // Configure IMU internal Bartlett filter
    writeRegister(kRegSENS_AVG, 1030);

    m_cmd = ByteBuffer.allocateDirect(26);
    m_cmd.put(0, (byte) kGLOB_CMD);
    m_cmd.put(1, (byte) 0);
    m_resp = ByteBuffer.allocateDirect(26);
    m_resp.order(ByteOrder.BIG_ENDIAN);

    // Configure interrupt on MXP DIO0
    m_interrupt = new DigitalInput(10);  // MXP DIO0
    m_task = new Thread(new ReadTask(this));
    m_interrupt.requestInterrupts();
    m_interrupt.setUpSourceEdge(false, true);
    m_task.setDaemon(true);
    m_task.start();

    calibrate();

    //UsageReporting.report(tResourceType.kResourceType_ADIS16448, 0);
    LiveWindow.addSensor("ADIS16448_IMU", 0, this);
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void calibrate() {
    if (m_spi == null) return;

    Timer.delay(0.1);

    synchronized (this) {
      m_accum_count = 0;
      m_accum_gyro_x = 0.0;
      m_accum_gyro_y = 0.0;
      m_accum_gyro_z = 0.0;
    }

    Timer.delay(kCalibrationSampleTime);

    synchronized (this) {
      m_gyro_center_x = m_accum_gyro_x / m_accum_count;
      m_gyro_center_y = m_accum_gyro_y / m_accum_count;
      m_gyro_center_z = m_accum_gyro_z / m_accum_count;
    }
  }

  private int readRegister(int reg) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    buf.order(ByteOrder.BIG_ENDIAN);
    buf.put(0, (byte) (reg & 0x7f));
    buf.put(1, (byte) 0);

    m_spi.write(buf, 2);
    m_spi.read(false, buf, 2);

    return ((int)buf.getShort(0)) & 0xffff;
  }

  private void writeRegister(int reg, int val) {
    ByteBuffer buf = ByteBuffer.allocateDirect(2);
    // low byte
    buf.put(0, (byte) (0x80 | reg));
    buf.put(1, (byte) val);
    m_spi.write(buf, 2);
    // high byte
    buf.put(0, (byte) (0x81 | reg));
    buf.put(1, (byte) (val >> 8));
    m_spi.write(buf, 2);
  }

  /**
   * {@inheritDoc}
   */
  public void reset() {
    synchronized (this) {
      m_integ_gyro_x = 0.0;
      m_integ_gyro_y = 0.0;
      m_integ_gyro_z = 0.0;
    }
  }

  /**
   * Delete (free) the spi port used for the IMU.
   */
  @Override
  public void free() {
    m_freed.set(true);
    try {
      m_task.join();
    } catch (InterruptedException e) {
    }
    if (m_interrupt != null) {
      m_interrupt.free();
      m_interrupt = null;
    }
    if (m_spi != null) {
      m_spi.free();
      m_spi = null;
    }
  }

  private void calculate() {
    synchronized (this) {
      m_last_sample_time = Timer.getFPGATimestamp();
    }
    while (!m_freed.get()) {
      if (m_interrupt.waitForInterrupt(kTimeout) ==
          InterruptableSensorBase.WaitResult.kTimeout)
        continue;

      double sample_time = m_interrupt.readFallingTimestamp();
      double dt;
      synchronized (this) {
        dt = sample_time - m_last_sample_time;
        m_last_sample_time = sample_time;
      }

      m_spi.transaction(m_cmd, m_resp, 26);

      double gyro_x = m_resp.getShort(4) * kDegreePerSecondPerLSB;
      double gyro_y = m_resp.getShort(6) * kDegreePerSecondPerLSB;
      double gyro_z = m_resp.getShort(8) * kDegreePerSecondPerLSB;
      double accel_x = m_resp.getShort(10) * kGPerLSB;
      double accel_y = m_resp.getShort(12) * kGPerLSB;
      double accel_z = m_resp.getShort(14) * kGPerLSB;
      double mag_x = m_resp.getShort(16) * kMilligaussPerLSB;
      double mag_y = m_resp.getShort(18) * kMilligaussPerLSB;
      double mag_z = m_resp.getShort(20) * kMilligaussPerLSB;

      // Make local copy of quaternion and angle global state
      double q1, q2, q3, q4;
      synchronized (this) {
        q1 = m_ahrs_q1;
        q2 = m_ahrs_q2;
        q3 = m_ahrs_q3;
        q4 = m_ahrs_q4;
      }

      // Kalman calculation
      // Code originated from: https://decibel.ni.com/content/docs/DOC-18964
      do {
        // If true, only use gyros and magnetos for updating the filter.
        boolean excludeAccel = false;

        // Convert accelerometer units to m/sec/sec
        double ax = accel_x * kAccelScale;
        double ay = accel_y * kAccelScale;
        double az = accel_z * kAccelScale;
        // Normalize accelerometer measurement
        double norm = Math.sqrt(ax * ax + ay * ay + az * az);
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
        norm = Math.sqrt(mx * mx + my * my + mz * mz);
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
        double _2bx = Math.sqrt(hx * hx + hy * hy);
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

        norm = Math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
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

        norm = Math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        if (norm > 0.0) {
          norm = 1.0 / norm;  // normalise quaternion
          q1 = q1 * norm;
          q2 = q2 * norm;
          q3 = q3 * norm;
          q4 = q4 * norm;
        }
      } while(false);

      // Convert quaternion to angles of rotation
      double xi = -Math.atan2(2*q2*q3 - 2*q1*q4, 2*(q1*q1) + 2*(q2*q2) - 1);
      double theta = -Math.asin(2*q2*q4 + 2*q1*q3);
      double rho = Math.atan2(2*q3*q4 - 2*q1*q2, 2*(q1*q1) + 2*(q4*q4) - 1);

      // Convert angles from radians to degrees
      xi = xi / Math.PI * 180.0;
      theta = theta / Math.PI * 180.0;
      rho = rho / Math.PI * 180.0;

      // Adjust angles for inverted mount of MXP sensor
      theta = -theta;
      if (rho < 0)
        rho = 180 - Math.abs(rho);
      else
        rho = Math.abs(rho) - 180;

      // Update global state
      synchronized (this) {
        m_gyro_x = gyro_x;
        m_gyro_y = gyro_y;
        m_gyro_z = gyro_z;
        m_accel_x = accel_x;
        m_accel_y = accel_y;
        m_accel_z = accel_z;
        m_mag_x = mag_x;
        m_mag_y = mag_y;
        m_mag_z = mag_z;
        m_baro = m_resp.getShort(22) * kMillibarPerLSB;
        m_temp = m_resp.getShort(24) * kDegCPerLSB + kDegCOffset;

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
  public double getAngle() {
    if (m_spi == null) return 0.0;
    return getYaw();
  }

  /**
   * {@inheritDoc}
   */
  public double getRate() {
    if (m_spi == null) return 0.0;
    return getRateZ();
  }

  public synchronized double getAngleX() {
    return m_integ_gyro_x;
  }

  public synchronized double getAngleY() {
    return m_integ_gyro_y;
  }

  public synchronized double getAngleZ() {
    return m_integ_gyro_z;
  }

  public synchronized double getRateX() {
    return m_gyro_x;
  }

  public synchronized double getRateY() {
    return m_gyro_y;
  }

  public synchronized double getRateZ() {
    return m_gyro_z;
  }

  public synchronized double getAccelX() {
    return m_accel_x;
  }

  public synchronized double getAccelY() {
    return m_accel_y;
  }

  public synchronized double getAccelZ() {
    return m_accel_z;
  }

  public synchronized double getMagX() {
    return m_mag_x;
  }

  public synchronized double getMagY() {
    return m_mag_y;
  }

  public synchronized double getMagZ() {
    return m_mag_z;
  }

  public synchronized double getPitch() {
    return m_pitch;
  }

  public synchronized double getRoll() {
    return m_roll;
  }

  public synchronized double getYaw() {
    return m_yaw;
  }

  public synchronized double getLastSampleTime() {
    return m_last_sample_time;
  }

  public synchronized double getBarometricPressure() {
    return m_baro;
  }

  public synchronized double getTemperature() {
    return m_temp;
  }

  public synchronized double getQuaternionW() {
    return m_ahrs_q1;
  }

  public synchronized double getQuaternionX() {
    return m_ahrs_q2;
  }

  public synchronized double getQuaternionY() {
    return m_ahrs_q3;
  }

  public synchronized double getQuaternionZ() {
    return m_ahrs_q4;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void updateTable() {
    ITable table = getTable();
    if (table != null) {
      table.putNumber("Value", getAngle());
      table.putNumber("Pitch", getPitch());
      table.putNumber("Roll", getRoll());
      table.putNumber("Yaw", getYaw());
      table.putNumber("AccelX", getAccelX());
      table.putNumber("AccelY", getAccelY());
      table.putNumber("AccelZ", getAccelZ());
      table.putNumber("AngleX", getAngleX());
      table.putNumber("AngleY", getAngleY());
      table.putNumber("AngleZ", getAngleZ());
    }
  }
}
