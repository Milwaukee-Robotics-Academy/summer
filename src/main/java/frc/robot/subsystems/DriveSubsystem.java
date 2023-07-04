// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder;
  private CANSparkMax m_rightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private RelativeEncoder m_rightEncoder;
  private AHRS m_gyro;
  private Rotation2d m_gyroOffset = new Rotation2d();

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  ShuffleboardTab fieldTab;
  ShuffleboardTab gyroTab;

  // Creating my kinematics object: track width of 27 inches
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.kTrackwidthMeters);
  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setInverted(true);

    m_rightMotor.setOpenLoopRampRate(0.4);
    m_leftMotor.setOpenLoopRampRate(0.4);

    double conversionFactor = 0.1524 * Math.PI / 10.71;
    //
    double velocityConversionFactor = (1 / 60.0) * (0.1524 * 2 * Math.PI) / 10.71;
    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setPositionConversionFactor(conversionFactor);
    m_leftEncoder.setVelocityConversionFactor(velocityConversionFactor);
    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setPositionConversionFactor(conversionFactor);
    m_rightEncoder.setVelocityConversionFactor(velocityConversionFactor);
    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    Shuffleboard.getTab("Field").add("Field", m_field);

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        getGyroRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition()); // Units.metersToInches(m_leftEncoder.getPosition()));
    SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition()); // Units.metersToInches(m_rightEncoder.getPosition()));
    SmartDashboard.putNumber("Average Distance",
        Units.metersToInches((m_rightEncoder.getPosition() + m_leftEncoder.getPosition()) / 2));
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("Gyro.getHeading", this.getGyroRotation2d().getDegrees());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    this.getGyroData();
    
  }

  private void getGyroData() {

   
    SmartDashboard.putData("Gyro", m_gyro);
    SmartDashboard.putNumber("getGyroRotation2d",
        this.getGyroRotation2d().getDegrees());
        SmartDashboard.putNumber("getYaw", m_gyro.getYaw());
        SmartDashboard.putNumber("getYaw through rotation2d", Rotation2d.fromDegrees(m_gyro.getYaw()).getDegrees());
        SmartDashboard.putNumber("getAngle", m_gyro.getAngle());
        SmartDashboard.putNumber("getAngleAdjustment", m_gyro.getAngleAdjustment());
        SmartDashboard.putNumber("m_gyroOffset.getDegrees()", m_gyroOffset.getDegrees());

     /* Display 6-axis Processed Angle Data                                      */
     SmartDashboard.putBoolean(  "IMU_Connected",        m_gyro.isConnected());
     SmartDashboard.putBoolean(  "IMU_IsCalibrating",    m_gyro.isCalibrating());
     SmartDashboard.putNumber(   "IMU_Yaw",              m_gyro.getYaw());
     SmartDashboard.putNumber(   "IMU_Pitch",            m_gyro.getPitch());
     SmartDashboard.putNumber(   "IMU_Roll",             m_gyro.getRoll());
     
     /* Display tilt-corrected, Magnetometer-based heading (requires             */
     /* magnetometer calibration to be useful)                                   */
     
     SmartDashboard.putNumber(   "IMU_CompassHeading",   m_gyro.getCompassHeading());
     
     /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
     SmartDashboard.putNumber(   "IMU_FusedHeading",     m_gyro.getFusedHeading());

     /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
     /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
     
     SmartDashboard.putNumber(   "IMU_TotalYaw",         m_gyro.getAngle());
     SmartDashboard.putNumber(   "IMU_YawRateDPS",       m_gyro.getRate());

     /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
     
     SmartDashboard.putNumber(   "IMU_Accel_X",          m_gyro.getWorldLinearAccelX());
     SmartDashboard.putNumber(   "IMU_Accel_Y",          m_gyro.getWorldLinearAccelY());
     SmartDashboard.putBoolean(  "IMU_IsMoving",         m_gyro.isMoving());
     SmartDashboard.putBoolean(  "IMU_IsRotating",       m_gyro.isRotating());

     /* Display estimates of velocity/displacement.  Note that these values are  */
     /* not expected to be accurate enough for estimating robot position on a    */
     /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
     /* of these errors due to single (velocity) integration and especially      */
     /* double (displacement) integration.                                       */
     
     SmartDashboard.putNumber(   "Velocity_X",           m_gyro.getVelocityX());
     SmartDashboard.putNumber(   "Velocity_Y",           m_gyro.getVelocityY());
     SmartDashboard.putNumber(   "Displacement_X",       m_gyro.getDisplacementX());
     SmartDashboard.putNumber(   "Displacement_Y",       m_gyro.getDisplacementY());
     
     /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
     /* NOTE:  These values are not normally necessary, but are made available   */
     /* for advanced users.  Before using this data, please consider whether     */
     /* the processed data (see above) will suit your needs.                     */
     
     SmartDashboard.putNumber(   "RawGyro_X",            m_gyro.getRawGyroX());
     SmartDashboard.putNumber(   "RawGyro_Y",            m_gyro.getRawGyroY());
     SmartDashboard.putNumber(   "RawGyro_Z",            m_gyro.getRawGyroZ());
     SmartDashboard.putNumber(   "RawAccel_X",           m_gyro.getRawAccelX());
     SmartDashboard.putNumber(   "RawAccel_Y",           m_gyro.getRawAccelY());
     SmartDashboard.putNumber(   "RawAccel_Z",           m_gyro.getRawAccelZ());
     SmartDashboard.putNumber(   "RawMag_X",             m_gyro.getRawMagX());
     SmartDashboard.putNumber(   "RawMag_Y",             m_gyro.getRawMagY());
     SmartDashboard.putNumber(   "RawMag_Z",             m_gyro.getRawMagZ());
     SmartDashboard.putNumber(   "IMU_Temp_C",           m_gyro.getTempC());
     
     /* Omnimount Yaw Axis Information                                           */
     /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
     AHRS.BoardYawAxis yaw_axis = m_gyro.getBoardYawAxis();
     SmartDashboard.putString(   "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
     SmartDashboard.putNumber(   "YawAxis",              yaw_axis.board_axis.getValue() );
     
     /* Sensor Board Information                                                 */
     SmartDashboard.putString(   "FirmwareVersion",      m_gyro.getFirmwareVersion());
     
     /* Quaternion Data                                                          */
     /* Quaternions are fascinating, and are the most compact representation of  */
     /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
     /* from the Quaternions.  If interested in motion processing, knowledge of  */
     /* Quaternions is highly recommended.                                       */
     SmartDashboard.putNumber(   "QuaternionW",          m_gyro.getQuaternionW());
     SmartDashboard.putNumber(   "QuaternionX",          m_gyro.getQuaternionX());
     SmartDashboard.putNumber(   "QuaternionY",          m_gyro.getQuaternionY());
     SmartDashboard.putNumber(   "QuaternionZ",          m_gyro.getQuaternionZ());
     
     /* Connectivity Debugging Support                                           */
     SmartDashboard.putNumber(   "IMU_Byte_Count",       m_gyro.getByteCount());
     SmartDashboard.putNumber(   "IMU_Update_Count",     m_gyro.getUpdateCount());
  }
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    zeroHeading(pose.getRotation());
    m_odometry.resetPosition(
        getGyroRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0.0);
    m_rightEncoder.setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading(Rotation2d rotation) {
    m_gyro.zeroYaw();
    m_gyroOffset = rotation;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading
   */
  public Rotation2d getGyroRotation2d() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).minus(m_gyroOffset);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public Command followTrajectoryCommand(ArrayList<PathPlannerTrajectory> path, HashMap<String, Command> eventMap, boolean isFirstPath) {
    SmartDashboard.putNumber("intial Pose rotation", path[0].getInitialPose().getRotation().getDegrees());
    m_field.getObject("traj").setTrajectory(path[0]);

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
  this::getPose, // Pose2d supplier
  this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
  new RamseteController(),
  this.kinematics,
  new SimpleMotorFeedforward(Constants.DriveConstants.kS, Constants.DriveConstants.kV,
      Constants.DriveConstants.kA),
  // DifferentialDriveKinematics
  this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
  new PIDConstants(Constants.DriveConstants.kP, 0, Constants.DriveConstants.kD), // Left controller. Tune
                                                                                  // these values for your
                                                                                  // robot. Leaving them 0
                                                                                  // will only use
                                                                                  // feedforwards.
   this::tankDriveVolts, // Voltage biconsumer
   eventMap,
  true, // Should the path be automatically mirrored depending on alliance color.
        // Optional, defaults to true
  this // Requires this drive subsystem
    );

    return autoBuilder.fullAuto(path);
    // return new SequentialCommandGroup(
    //     new InstantCommand(() -> {
    //       // Reset odometry for the first path you run during auto
    //       if (isFirstPath) {
    //         this.resetOdometry(path[0].getInitialPose());
    //       }
    //     }),
    //     autoBuilder.fullAuto(path));
  }
}
