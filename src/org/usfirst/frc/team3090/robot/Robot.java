package org.usfirst.frc.team3090.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program providing a real-time display of navX-MXP values.
 *
 * In the operatorControl() method, all data from the navX-MXP is retrieved
 * and output to the SmartDashboard.
 *
 * The output data values include:
 *
 * - Yaw, Pitch and Roll angles
 * - Compass Heading and 9-Axis Fused Heading (requires Magnetometer calibration)
 * - Linear Acceleration Data
 * - Motion Indicators
 * - Estimated Velocity and Displacement
 * - Quaternion Data
 * - Raw Gyro, Accelerometer and Magnetometer Data
 *
 * As well, Board Information is also retrieved; this can be useful for debugging
 * connectivity issues after initial installation of the navX-MXP sensor.
 *
 */
public class Robot extends SampleRobot {
  AHRS ahrs;
  Joystick stick;
  DecimalFormat df;
    
  public Robot() {
      df = new DecimalFormat("###.##");
      stick = new Joystick(0);
      try {
          /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
          /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
          /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
          ahrs = new AHRS(I2C.Port.kMXP); 
      } catch (RuntimeException ex ) {
          DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
  }

  /**
   * Runs during autonomous mode
   */
  public void autonomous() {
      Timer.delay(2.0);		//    for 2 seconds
  }


  /**
   * Display navX-MXP Sensor Data on Smart Dashboard
   */
  public void operatorControl() {
      while (isOperatorControl() && isEnabled()) {
          
          Timer.delay(0.020);		/* wait for one motor update time period (50Hz)     */
          
          boolean zero_yaw_pressed = stick.getTrigger();
          if ( zero_yaw_pressed ) {
              ahrs.zeroYaw();
          }

          /* Display 6-axis Processed Angle Data                                      */
          SmartDashboard.putBoolean(  "IMU_Connected",        df.format(ahrs.isConnected()));
          SmartDashboard.putBoolean(  "IMU_IsCalibrating",    df.format(ahrs.isCalibrating()));
          SmartDashboard.putNumber(   "IMU_Yaw",              df.format(ahrs.getYaw()));
          SmartDashboard.putNumber(   "IMU_Pitch",            df.format(ahrs.getPitch()));
          SmartDashboard.putNumber(   "IMU_Roll",             df.format(ahrs.getRoll()));
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */
          
          SmartDashboard.putNumber(   "IMU_CompassHeading",   df.format(ahrs.getCompassHeading()));
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          SmartDashboard.putNumber(   "IMU_FusedHeading",     df.format(ahrs.getFusedHeading()));

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
          
          SmartDashboard.putNumber(   "IMU_TotalYaw",         df.format(ahrs.getAngle()));
          SmartDashboard.putNumber(   "IMU_YawRateDPS",       df.format(ahrs.getRate()));

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          SmartDashboard.putNumber(   "IMU_Accel_X",          df.format(ahrs.getWorldLinearAccelX()));
          SmartDashboard.putNumber(   "IMU_Accel_Y",          df.format(ahrs.getWorldLinearAccelY()));
          SmartDashboard.putBoolean(  "IMU_IsMoving",         df.format(ahrs.isMoving()));
          SmartDashboard.putBoolean(  "IMU_IsRotating",       df.format(ahrs.isRotating()));

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          SmartDashboard.putNumber(   "Velocity_X",           df.format(ahrs.getVelocityX()));
          SmartDashboard.putNumber(   "Velocity_Y",           df.format(ahrs.getVelocityY()));
          SmartDashboard.putNumber(   "Displacement_X",       df.format(ahrs.getDisplacementX()));
          SmartDashboard.putNumber(   "Displacement_Y",       df.format(ahrs.getDisplacementY()));
          
          /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
          /* NOTE:  These values are not normally necessary, but are made available   */
          /* for advanced users.  Before using this data, please consider whether     */
          /* the processed data (see above) will suit your needs.                     */
          
          SmartDashboard.putNumber(   "RawGyro_X",            df.format(ahrs.getRawGyroX()));
          SmartDashboard.putNumber(   "RawGyro_Y",            df.format(ahrs.getRawGyroY()));
          SmartDashboard.putNumber(   "RawGyro_Z",            df.format(ahrs.getRawGyroZ()));
          SmartDashboard.putNumber(   "RawAccel_X",           df.format(ahrs.getRawAccelX()));
          SmartDashboard.putNumber(   "RawAccel_Y",           df.format(ahrs.getRawAccelY()));
          SmartDashboard.putNumber(   "RawAccel_Z",           df.format(ahrs.getRawAccelZ()));
          SmartDashboard.putNumber(   "RawMag_X",             df.format(ahrs.getRawMagX()));
          SmartDashboard.putNumber(   "RawMag_Y",             df.format(ahrs.getRawMagY()));
          SmartDashboard.putNumber(   "RawMag_Z",             df.format(ahrs.getRawMagZ()));
          SmartDashboard.putNumber(   "IMU_Temp_C",           df.format(ahrs.getTempC()));
          
          /* Omnimount Yaw Axis Information                                           */
          /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
          AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
          SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
          SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
          
          /* Sensor Board Information                                                 */
          SmartDashboard.putString(   "FirmwareVersion",      df.format(ahrs.getFirmwareVersion()));
          
          /* Quaternion Data                                                          */
          /* Quaternions are fascinating, and are the most compact representation of  */
          /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
          /* from the Quaternions.  If interested in motion processing, knowledge of  */
          /* Quaternions is highly recommended.                                       */
          SmartDashboard.putNumber(   "QuaternionW",          df.format(ahrs.getQuaternionW()));
          SmartDashboard.putNumber(   "QuaternionX",          df.format(ahrs.getQuaternionX()));
          SmartDashboard.putNumber(   "QuaternionY",          df.format(ahrs.getQuaternionY()));
          SmartDashboard.putNumber(   "QuaternionZ",          df.format(ahrs.getQuaternionZ()));
          
          /* Connectivity Debugging Support                                           */
          SmartDashboard.putNumber(   "IMU_Byte_Count",       df.format(ahrs.getByteCount()));
          SmartDashboard.putNumber(   "IMU_Update_Count",     df.format(ahrs.getUpdateCount()));
      }
  }

  /**
   * Runs during test mode
   */
  public void test() {
  }
}
