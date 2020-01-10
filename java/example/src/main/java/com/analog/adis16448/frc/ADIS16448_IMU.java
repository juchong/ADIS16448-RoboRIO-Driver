/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2016. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.analog.adis16448.frc;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class is for the ADIS16448 IMU that connects to the RoboRIO MXP port.
 */
@SuppressWarnings("unused")
public class ADIS16448_IMU extends GyroBase implements Gyro, PIDSource, Sendable {
	private static final double kCalibrationSampleTime = 5.0; // Calibration time in seconds
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
  //private static final int kRegLOT_ID2 = 0x54;
  //private static final int kRegLOT_ID1 = 0x52;
  //private static final int kRegSERIAL_NUM = 0x58;
  //private static final int kRegZGYRO_OFF = 0x1E;
  //private static final int kRegYGYRO_OFF = 0x1C;
  private static final int kRegXGYRO_OFF = 0x1A;

  public enum AHRSAlgorithm { kComplementary, kMadgwick }
  public enum Axis { kX, kY, kZ }

  // AHRS algorithm
  private AHRSAlgorithm m_algorithm;

  // AHRS yaw axis
  private Axis m_yaw_axis;

	//CRC-16 Look-Up Table
	int adiscrc[] = new int[]{
	0x0000, 0x17CE, 0x0FDF, 0x1811, 0x1FBE, 0x0870, 0x1061, 0x07AF,
	0x1F3F, 0x08F1, 0x10E0, 0x072E, 0x0081, 0x174F, 0x0F5E, 0x1890,
	0x1E3D, 0x09F3, 0x11E2, 0x062C, 0x0183, 0x164D, 0x0E5C, 0x1992,
	0x0102, 0x16CC, 0x0EDD, 0x1913, 0x1EBC, 0x0972, 0x1163, 0x06AD,
	0x1C39, 0x0BF7, 0x13E6, 0x0428, 0x0387, 0x1449, 0x0C58, 0x1B96,
	0x0306, 0x14C8, 0x0CD9, 0x1B17, 0x1CB8, 0x0B76, 0x1367, 0x04A9,
	0x0204, 0x15CA, 0x0DDB, 0x1A15, 0x1DBA, 0x0A74, 0x1265, 0x05AB,
	0x1D3B, 0x0AF5, 0x12E4, 0x052A, 0x0285, 0x154B, 0x0D5A, 0x1A94,
	0x1831, 0x0FFF, 0x17EE, 0x0020, 0x078F, 0x1041, 0x0850, 0x1F9E,
	0x070E, 0x10C0, 0x08D1, 0x1F1F, 0x18B0, 0x0F7E, 0x176F, 0x00A1,
	0x060C, 0x11C2, 0x09D3, 0x1E1D, 0x19B2, 0x0E7C, 0x166D, 0x01A3,
	0x1933, 0x0EFD, 0x16EC, 0x0122, 0x068D, 0x1143, 0x0952, 0x1E9C,
	0x0408, 0x13C6, 0x0BD7, 0x1C19, 0x1BB6, 0x0C78, 0x1469, 0x03A7,
	0x1B37, 0x0CF9, 0x14E8, 0x0326, 0x0489, 0x1347, 0x0B56, 0x1C98,
	0x1A35, 0x0DFB, 0x15EA, 0x0224, 0x058B, 0x1245, 0x0A54, 0x1D9A,
	0x050A, 0x12C4, 0x0AD5, 0x1D1B, 0x1AB4, 0x0D7A, 0x156B, 0x02A5,
	0x1021, 0x07EF, 0x1FFE, 0x0830, 0x0F9F, 0x1851, 0x0040, 0x178E,
	0x0F1E, 0x18D0, 0x00C1, 0x170F, 0x10A0, 0x076E, 0x1F7F, 0x08B1,
	0x0E1C, 0x19D2, 0x01C3, 0x160D, 0x11A2, 0x066C, 0x1E7D, 0x09B3,
	0x1123, 0x06ED, 0x1EFC, 0x0932, 0x0E9D, 0x1953, 0x0142, 0x168C,
	0x0C18, 0x1BD6, 0x03C7, 0x1409, 0x13A6, 0x0468, 0x1C79, 0x0BB7,
	0x1327, 0x04E9, 0x1CF8, 0x0B36, 0x0C99, 0x1B57, 0x0346, 0x1488,
	0x1225, 0x05EB, 0x1DFA, 0x0A34, 0x0D9B, 0x1A55, 0x0244, 0x158A,
	0x0D1A, 0x1AD4, 0x02C5, 0x150B, 0x12A4, 0x056A, 0x1D7B, 0x0AB5,
	0x0810, 0x1FDE, 0x07CF, 0x1001, 0x17AE, 0x0060, 0x1871, 0x0FBF,
	0x172F, 0x00E1, 0x18F0, 0x0F3E, 0x0891, 0x1F5F, 0x074E, 0x1080,
	0x162D, 0x01E3, 0x19F2, 0x0E3C, 0x0993, 0x1E5D, 0x064C, 0x1182,
	0x0912, 0x1EDC, 0x06CD, 0x1103, 0x16AC, 0x0162, 0x1973, 0x0EBD,
	0x1429, 0x03E7, 0x1BF6, 0x0C38, 0x0B97, 0x1C59, 0x0448, 0x1386,
	0x0B16, 0x1CD8, 0x04C9, 0x1307, 0x14A8, 0x0366, 0x1B77, 0x0CB9,
	0x0A14, 0x1DDA, 0x05CB, 0x1205, 0x15AA, 0x0264, 0x1A75, 0x0DBB,
	0x152B, 0x02E5, 0x1AF4, 0x0D3A, 0x0A95, 0x1D5B, 0x054A, 0x1284
	};

  // serial number and lot id
  //private int m_serial_num;
  //private int m_lot_id1;
  //private int m_lot_id2;

  // gyro offset
  private double m_gyro_offset_x = 0.0;
  private double m_gyro_offset_y = 0.0;
  private double m_gyro_offset_z = 0.0;

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

  // Complementary AHRS
  private boolean m_first = true;
  private double m_gyro_x_prev;
  private double m_gyro_y_prev;
  private double m_gyro_z_prev;
  private double m_mag_angle_prev = 0.0;
  private boolean m_tilt_comp_yaw = true;

  // AHRS outputs
  private double m_yaw = 0.0;
  private double m_roll = 0.0;
  private double m_pitch = 0.0;

  private AtomicBoolean m_freed = new AtomicBoolean(false);

  private SPI m_spi;
  private DigitalInput m_interrupt;

  // Sample from the IMU
  private static class Sample {
    public double gyro_x;
    public double gyro_y;
    public double gyro_z;
    public double accel_x;
    public double accel_y;
    public double accel_z;
    public double mag_x;
    public double mag_y;
    public double mag_z;
    public double baro;
    public double temp;
    public double dt;

    // Swap axis as appropriate for yaw axis selection
    public void adjustYawAxis(Axis yaw_axis) {
      switch (yaw_axis) {
        case kX: {
          // swap X and Z
          double tmp;
          tmp = accel_x;
          accel_x = accel_z;
          accel_z = tmp;
          tmp = mag_x;
          mag_x = mag_z;
          mag_z = tmp;
          tmp = gyro_x;
          gyro_x = gyro_z;
          gyro_z = tmp;
          break;
        }
        case kY: {
          // swap Y and Z
          double tmp;
          tmp = accel_y;
          accel_y = accel_z;
          accel_z = tmp;
          tmp = mag_y;
          mag_y = mag_z;
          mag_z = tmp;
          tmp = gyro_y;
          gyro_y = gyro_z;
          gyro_z = tmp;
          break;
        }
        case kZ:
        default:
          // no swap required
          break;
      }
    }
  }

  // Sample FIFO
  private static final int kSamplesDepth = 10;
  private final Sample[] m_samples;
  private final Lock m_samples_mutex;
  private final Condition m_samples_not_empty;
  private int m_samples_count = 0;
  private int m_samples_take_index = 0;
  private int m_samples_put_index = 0;
  private boolean m_calculate_started = false;

  // Previous timestamp
  long timestamp_old = 0;

  private static class AcquireTask implements Runnable {
    private ADIS16448_IMU imu;
    public AcquireTask(ADIS16448_IMU imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.acquire();
    }
  }
  private static class CalculateTask implements Runnable {
    private ADIS16448_IMU imu;
    public CalculateTask(ADIS16448_IMU imu) {
      this.imu = imu;
    }

    @Override
    public void run() {
      imu.calculate();
    }
  }
  private Thread m_acquire_task;
  private Thread m_calculate_task;

  /**
   * @param yaw_axis Which axis is Yaw
   * @param algorithm Use {@link #calculateComplementary} or {@link #calculateMadgwick} algorithm
   */
  public ADIS16448_IMU(Axis yaw_axis, AHRSAlgorithm algorithm) {
    m_yaw_axis = yaw_axis;
    m_algorithm = algorithm;

    // Force the IMU reset pin to toggle on startup (doesn't require DS enable)
    DigitalOutput m_reset_out = new DigitalOutput(18);  // Drive MXP DIO8 low
    Timer.delay(0.01);  // Wait 10ms
    m_reset_out.close();
    DigitalInput m_reset_in = new DigitalInput(18);  // Set MXP DIO8 high
    Timer.delay(0.5);  // Wait 500ms

    m_spi = new SPI(SPI.Port.kMXP);
    m_spi.setClockRate(1000000);
    m_spi.setMSBFirst();
    m_spi.setSampleDataOnFalling();
    m_spi.setClockActiveLow();
    m_spi.setChipSelectActiveLow();

    readRegister(kRegPROD_ID); // dummy read

    // Validate the product ID
    if (readRegister(kRegPROD_ID) != 16448) {
      m_spi.close();
      m_spi = null;
      m_samples = null;
      m_samples_mutex = null;
      m_samples_not_empty = null;
      DriverStation.reportError("could not find ADIS16448", false);
      return;
    }

    // Set IMU internal decimation to 102.4 SPS
    writeRegister(kRegSMPL_PRD, 0x0301);

    // Enable Data Ready (LOW = Good Data) on DIO1 (PWM0 on MXP) & PoP
    writeRegister(kRegMSC_CTRL, 0x0056);

    // Configure IMU internal Bartlett filter
    writeRegister(kRegSENS_AVG, 0x0402);

    // Read serial number and lot ID
    //m_serial_num = readRegister(kRegSERIAL_NUM);
    //m_lot_id2 = readRegister(kRegLOT_ID2);
    //m_lot_id1 = readRegister(kRegLOT_ID1);

    // Create data acq FIFO.  We make the FIFO 2 longer than it needs
    // to be so the input and output never overlap (we hold a reference
    // to the output while the lock is released).
    m_samples_mutex = new ReentrantLock();
    m_samples_not_empty = m_samples_mutex.newCondition();

    m_samples = new Sample[kSamplesDepth + 2];
    for (int i=0; i<kSamplesDepth + 2; i++) {
      m_samples[i] = new Sample();
    }

    // Configure interrupt on MXP DIO0
    m_interrupt = new DigitalInput(10);
    // Configure SPI bus for DMA read
    m_spi.initAuto(8200);
    m_spi.setAutoTransmitData(new byte[] {kGLOB_CMD},27);
    m_spi.startAutoTrigger(m_interrupt, true, false);

    m_freed.set(false);
    m_acquire_task = new Thread(new AcquireTask(this));
    m_acquire_task.setDaemon(true);
    m_acquire_task.start();

    // Start AHRS processing
    m_calculate_task = new Thread(new CalculateTask(this));
    m_calculate_task.setDaemon(true);
    m_calculate_task.start();

    calibrate();

    // Report usage and post data to DS

    HAL.report(tResourceType.kResourceType_ADIS16448, 0);
    setName("ADIS16448_IMU");
  }

  /*
   * Constructor assuming Complementary AHRS algorithm.
   */
  public ADIS16448_IMU(Axis yaw_axis) {
    this(yaw_axis, AHRSAlgorithm.kComplementary);
  }

  /*
   * Constructor assuming yaw axis is "Z" and Complementary AHRS algorithm.
   */
  public ADIS16448_IMU() {
    this(Axis.kZ, AHRSAlgorithm.kComplementary);
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
      m_gyro_offset_x = m_accum_gyro_x / m_accum_count;
      m_gyro_offset_y = m_accum_gyro_y / m_accum_count;
      m_gyro_offset_z = m_accum_gyro_z / m_accum_count;
    }
  }

  static int ToUShort(ByteBuffer buf) {
	  return (buf.getShort(0)) & 0xFFFF;
  }
  static int ToUShort(byte[] buf) {
    return (((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }  
  static int ToUShort(int... data) {
	  byte[] buf = new byte[data.length];
	  for(int i = 0; i < data.length; ++i) {
		  buf[i] = (byte)data[i];
	  }
	  return ToUShort(buf);
  }

  public static long ToULong(int sint) {
		return sint & 0x00000000FFFFFFFFL;
	}

  private static int ToShort(int... buf) {
    return (short)(((buf[0] & 0xFF) << 8) + ((buf[1] & 0xFF) << 0));
  }
  static int ToShort(ByteBuffer buf) {
	  return ToShort(buf.get(0), buf.get(1));
  }

  static int ToShort(byte[] buf) {
    return buf[0] << 8 | buf[1];
  }

  private int readRegister(int reg) {
    //ByteBuffer buf = ByteBuffer.allocateDirect(2);
    byte[] buf = new byte[2];
    //buf.order(ByteOrder.BIG_ENDIAN);
    buf[0] = (byte) (reg & 0x7f);
    buf[1] = (byte) 0;

    m_spi.write(buf, 2);
    m_spi.read(false, buf, 2);

    return ToUShort(buf);
  }

  private void writeRegister(int reg, int val) {
    //ByteBuffer buf = ByteBuffer.allocateDirect(2);
    byte[] buf = new byte[2];
    // low byte
    buf[0] = (byte)((0x80 | reg) | 0x10);
    buf[1] = (byte) (val & 0xff);
    m_spi.write(buf, 2);
    // high byte
    buf[0] = (byte) (0x81 | reg);
    buf[1] =(byte) (val >> 8);
    m_spi.write(buf, 2);
  }

  private void printBytes(int[] data) {
		for(int i = 0; i < data.length; ++i) {
			System.out.print(data[i] + " ");
		}
		System.out.println();
	}
  private void printBytes(byte[] data) {
		for(int i = 0; i < data.length; ++i) {
			System.out.print(data[i] + " ");
		}
		System.out.println();
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
  public void close() {
    m_freed.set(true);
    if (m_samples_mutex != null) {
      m_samples_mutex.lock();
      try {
        m_samples_not_empty.signal();
      } finally {
        m_samples_mutex.unlock();
      }
    }
    try {
      if (m_acquire_task != null) {
        m_acquire_task.join();
      }
      if (m_calculate_task != null) {
        m_calculate_task.join();
      }
    } catch (InterruptedException e) {
    }
    if (m_interrupt != null) {
      m_interrupt.close();
      m_interrupt = null;
    }
    if (m_spi != null) {
      m_spi.close();
      m_spi = null;
    }
  }

  private void acquire() {
    int[] readBuf = new int[2000];
    //ByteBuffer readBuf = ByteBuffer.allocateDirect(64000);
    //readBuf.order(ByteOrder.LITTLE_ENDIAN);
    double gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z, baro, temp;
    int data_count = 0;
    int array_offset = 0;
    int imu_crc = 0;
    double dt = 0; // This number must be adjusted if decimation setting is changed. Default is 1/102.4 SPS
    int data_subset[] = new int[28];
    byte bBuf[] = new byte[2];
    long timestamp_new = 0;
    int data_to_read = 0;

    while (!m_freed.get()) {
      // Waiting for the buffer to fill...
  	  try{Thread.sleep(20);}catch(InterruptedException e){} // A delay less than 10ms could potentially overflow the local buffer

  	  data_count = m_spi.readAutoReceivedData(readBuf,0,0); // Read number of bytes currently stored in the buffer
      array_offset = data_count % 116; // Look for "extra" data This is 116 not 29 like in C++ b/c everything is 32-bits and takes up 4 bytes in the buffer
      data_to_read = data_count - array_offset; // Discard "extra" data
      m_spi.readAutoReceivedData(readBuf,data_to_read,0); // Read data from DMA buffer
      for(int i = 0; i < data_to_read; i += 116) { // Process each set of 28 bytes (timestamp + 28 data) * 4 (32-bit ints)
        
        for(int j = 1; j < 29; j++) { // Split each set of 28 bytes into a sub-array for processing
          int at  = (i + 1 * (j));
			    data_subset[j - 1] = readBuf[at];//readBuf.getInt(at);
        }

        // DEBUG: Print the received data
        //printBytes(data_subset);

        // DEBUG: Plot Sub-Array Data in Terminal
        /*System.out.println(ToUShort(data_subset[0], data_subset[1]) + "," + ToUShort(data_subset[2], data_subset[3]) + "," +
        ToUShort(data_subset[4], data_subset[5]) + "," + ToUShort(data_subset[6], data_subset[7]) + "," + ToUShort(data_subset[8], data_subset[9]) + ","
        + ToUShort(data_subset[10], data_subset[11]) + "," +
        ToUShort(data_subset[12], data_subset[13]) + "," + ToUShort(data_subset[14], data_subset[15]) + ","
        + ToUShort(data_subset[16], data_subset[17]) + "," +
        ToUShort(data_subset[18], data_subset[19]) + "," + ToUShort(data_subset[20], data_subset[21]) + ","
        + ToUShort(data_subset[22], data_subset[23]) + "," +
        ToUShort(data_subset[24], data_subset[25]) + "," + ToUShort(data_subset[26], data_subset[27]));*/

        // Calculate CRC-16 on each data packet
        int calc_crc = 0x0000FFFF; // Starting word
        int read_byte = 0;
        for(int k = 4; k < 26; k += 2 ) { // Cycle through XYZ GYRO, XYZ ACCEL, XYZ MAG, BARO, TEMP (Ignore Status & CRC)
          read_byte = data_subset[k+1]; // Process LSB
          calc_crc = (calc_crc >>> 8) ^ adiscrc[(calc_crc & 0x000000FF) ^ read_byte];
          read_byte = data_subset[k]; // Process MSB
          calc_crc = (calc_crc >>> 8) ^ adiscrc[(calc_crc & 0x000000FF) ^ read_byte];
        }

        // Make sure to mask all but relevant 16 bits
        calc_crc = ~calc_crc & 0xFFFF;
        calc_crc = ((calc_crc << 8) | (calc_crc >> 8)) & 0xFFFF;
        //System.out.println("Calc: " + calc_crc);

        // This is the data needed for CRC
        //ByteBuffer bBuf = ByteBuffer.allocateDirect(2);
        //byte[] bBuf = new byte[2];
        bBuf[0] = (byte)data_subset[26]; 
        bBuf[1] = (byte)data_subset[27];

        //System.out.println("Data: " + bBuf[0] + "," + bBuf[1]);

        imu_crc = ToUShort(bBuf); // Extract DUT CRC from data
        //System.out.println("IMU: " + imu_crc);
        //System.out.println("------------");

        // Compare calculated vs read CRC. Don't update outputs if CRC-16 is bad
        if(calc_crc == imu_crc) {
          // Calculate delta-time (dt) using FPGA timestamps
          timestamp_new = ToULong(readBuf[i]);
          dt = (timestamp_new - timestamp_old)/1000000.0; // Calculate dt and convert us to seconds
          timestamp_old = timestamp_new; // Store new timestamp in old variable for next cycle

          gyro_x = ToShort(data_subset[4], data_subset[5]) * kDegreePerSecondPerLSB;
          gyro_y = ToShort(data_subset[6], data_subset[7]) * kDegreePerSecondPerLSB;
          gyro_z = ToShort(data_subset[8], data_subset[9]) * kDegreePerSecondPerLSB;
          accel_x = ToShort(data_subset[10], data_subset[11]) * kGPerLSB;
          accel_y = ToShort(data_subset[12], data_subset[13]) * kGPerLSB;
          accel_z = ToShort(data_subset[14], data_subset[15]) * kGPerLSB;
          mag_x = ToShort(data_subset[16], data_subset[17]) * kMilligaussPerLSB;
          mag_y = ToShort(data_subset[18], data_subset[19]) * kMilligaussPerLSB;
          mag_z = ToShort(data_subset[20], data_subset[21]) * kMilligaussPerLSB;
          baro = ToUShort(data_subset[22], data_subset[23]) * kMillibarPerLSB;
          temp = ToShort(data_subset[24], data_subset[25]) * kDegCPerLSB + kDegCOffset;

          // Print scaled data to terminal
          /*System.out.println(gyro_x + "," + gyro_y + "," + gyro_z + "," + accel_x + "," + accel_y + ","
          + accel_z + "," + mag_x + "," + mag_y + "," + mag_z + "," + baro + "," + temp + "," + ","
          + ToUShort(data_subset[26], data_subset[27]));*/
          //System.out.println("---------------------"); // Frame divider (or else data looks like a mess)

          m_samples_mutex.lock();
          try{
            // If the FIFO is full, just drop it
            if (m_calculate_started && m_samples_count < kSamplesDepth)
            {
              Sample sample = m_samples[m_samples_put_index];
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
              m_samples_not_empty.signal();
            }
          }catch(Exception e) {
            break;
          }finally {
            m_samples_mutex.unlock();
          }

          // Update global state
          synchronized(this){
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
        }else{
          System.out.println("Invalid CRC");
        }
	    }
    }
  }

  private void calculate() {
    while (!m_freed.get()) {
      // Wait for next sample and get it
      try{Thread.sleep(20);}catch(InterruptedException e){}
      Sample sample;
      m_samples_mutex.lock();
      try {
        m_calculate_started = true;
        while (m_samples_count == 0) {
          m_samples_not_empty.await();
          if (m_freed.get()) {
            return;
          }
        }
        sample = m_samples[m_samples_take_index];
        ++m_samples_take_index;
        if (m_samples_take_index == (kSamplesDepth + 2))
          m_samples_take_index = 0;
        --m_samples_count;
      } catch (InterruptedException e) {
        break;
      } finally {
        m_samples_mutex.unlock();
      }

      switch (m_algorithm) {
        case kMadgwick:
          calculateMadgwick(sample, 0.4);
          break;
        case kComplementary:
        default:
          calculateComplementary(sample);
          break;
      }
    }
  }

  private void calculateMadgwick(Sample sample, double beta) {
    // Make local copy of quaternion and angle global state
    double q1, q2, q3, q4;
    synchronized (this) {
      q1 = m_ahrs_q1;
      q2 = m_ahrs_q2;
      q3 = m_ahrs_q3;
      q4 = m_ahrs_q4;
    }

    // Swap axis as appropriate for yaw axis selection
    sample.adjustYawAxis(m_yaw_axis);

    // Kalman calculation
    // Code originated from: https://decibel.ni.com/content/docs/DOC-18964
    do {
      // If true, only use gyros and magnetos for updating the filter.
      boolean excludeAccel = false;

      // Convert accelerometer units to m/sec/sec
      double ax = sample.accel_x * kAccelScale;
      double ay = sample.accel_y * kAccelScale;
      double az = sample.accel_z * kAccelScale;
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
      double mx = sample.mag_x * kMagScale;
      double my = sample.mag_y * kMagScale;
      double mz = sample.mag_z * kMagScale;
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
  private void calculateComplementary(Sample sample) {
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
    sample.adjustYawAxis(m_yaw_axis);

    final double tau_acc = 0.95;
    final double tau_mag = 0.04;

    double roll, pitch, yaw;
    boolean tilt_comp_yaw;
    synchronized (this) {
      roll = m_roll;
      pitch = m_pitch;
      yaw = m_yaw;
      tilt_comp_yaw = m_tilt_comp_yaw;
    }

    // Calculate mag angle in degrees
    double mag_angle = Math.atan2(sample.mag_y, sample.mag_x) / Math.PI * 180.0;

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
    double tilt_pitch_rad = Math.atan2(-sample.accel_x, Math.sqrt(sample.accel_y * sample.accel_y + sample.accel_z * sample.accel_z));
    double tilt_pitch = tilt_pitch_rad / Math.PI * 180.0;

    double tilt_roll_rad = Math.atan2(sample.accel_y, Math.sqrt(sample.accel_x * sample.accel_x * 0.01 + sample.accel_z * sample.accel_z) * Math.signum(sample.accel_z));
    double tilt_roll = tilt_roll_rad / Math.PI * 180.0;

    // Mag
    double tilt_yaw;
    if (tilt_comp_yaw) {
      double sin_pitch = Math.sin(tilt_pitch_rad);
      double cos_pitch = Math.cos(tilt_pitch_rad);
      double sin_roll = Math.sin(tilt_roll_rad);
      double cos_roll = Math.cos(tilt_roll_rad);
      double mx2 = sample.mag_x * cos_pitch + sample.mag_z * sin_pitch;
      double my2 = sample.mag_x * sin_roll * sin_pitch + sample.mag_y * cos_roll - sample.mag_z * sin_roll * cos_pitch;
      //double mz2 = -sample.mag_x * cos_roll * sin_pitch + sample.mag_y * sin_roll + sample.mag_z * cos_roll * cos_pitch;
      tilt_yaw = Math.atan2(my2, mx2) / Math.PI * 180.0;
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
    if (Math.abs(mag_angle - m_mag_angle_prev) >= 180) {
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
    synchronized (this) {
      m_roll = roll;
      m_pitch = pitch;
      m_yaw = yaw;
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

  // Get quaternion W for the Kalman AHRS.
  // Always returns 0 for the Complementary AHRS.
  public synchronized double getQuaternionW() {
    return m_ahrs_q1;
  }

  // Get quaternion X for the Kalman AHRS.
  // Always returns 0 for the Complementary AHRS.
  public synchronized double getQuaternionX() {
    return m_ahrs_q2;
  }

  // Get quaternion Y for the Kalman AHRS.
  // Always returns 0 for the Complementary AHRS.
  public synchronized double getQuaternionY() {
    return m_ahrs_q3;
  }

  // Get quaternion Z for the Kalman AHRS.
  // Always returns 0 for the Complementary AHRS.
  public synchronized double getQuaternionZ() {
    return m_ahrs_q4;
  }

  // Enable or disable yaw tilt-compensation for the Complementary AHRS.
  // Has no effect on the Kalman AHRS.
  //
  // It is likely best to set Yaw tilt-compensation to off (False) if the Yaw
  // value is to be used as feedback in a closed loop control application.
  // The tradeoff is that Yaw will only be accurate while the robot is level.
  public synchronized void setTiltCompYaw(boolean enabled) {
    m_tilt_comp_yaw = enabled;
  }

  /**
   * {@inheritDoc}
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Value", ()-> getAngle(), null);
    builder.addDoubleProperty("Pitch", ()-> getPitch(), null);
    builder.addDoubleProperty("Roll", ()-> getRoll(), null);
    builder.addDoubleProperty("Yaw", ()-> getYaw(), null);
    builder.addDoubleProperty("AccelX", ()-> getAccelX(), null);
    builder.addDoubleProperty("AccelY", ()-> getAccelY(), null);
    builder.addDoubleProperty("AccelZ", ()-> getAccelZ(), null);
    builder.addDoubleProperty("AngleX", ()-> getAngleX(), null);
    builder.addDoubleProperty("AngleY", ()-> getAngleY(), null);
    builder.addDoubleProperty("AngleZ", ()-> getAngleZ(), null);
  }

}
