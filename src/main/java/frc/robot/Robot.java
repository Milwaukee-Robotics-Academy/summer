// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs
 * the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  private CANSparkMax m_leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder;
  private CANSparkMax m_rightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private RelativeEncoder m_rightEncoder;
  private DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private XboxController m_stick = new XboxController(0);
  private AHRS gyro;
  private Timer autoTimer = new Timer();

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    // gyro = new AHRS(SPI.Port.kMXP);

    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setInverted(true);

    double conversionFactor = 0.1524 * Math.PI * 10.71;
    m_leftEncoder = m_leftMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    m_leftEncoder.setPositionConversionFactor(conversionFactor);
    m_rightEncoder = m_rightMotor.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 4096);
    m_rightEncoder.setPositionConversionFactor(conversionFactor);

  }

  @Override
  public void teleopInit() {
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(m_stick.getRightTriggerAxis() - m_stick.getLeftTriggerAxis(), -m_stick.getLeftX());

    SmartDashboard.putNumber("Left Encoder", Units.metersToInches(m_leftEncoder.getPosition()));
    SmartDashboard.putNumber("Right Encoder", Units.metersToInches(m_rightEncoder.getPosition()));
    SmartDashboard.putNumber("Average Distance", getAverageDistanceInInches());
    // SmartDashboard.putData("Gyro", gyro);

  }

  double getAverageDistanceInInches() {
    return Units.metersToInches((m_rightEncoder.getPosition() + m_leftEncoder.getPosition()) / 2);
  }

  public void autonomousInit() {

    autoTimer.reset();
    autoTimer.start();
    // ahrs.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (autoTimer.get() < 5) {
      if (getAverageDistanceInInches() > 24) {
        m_robotDrive.tankDrive(0.5, 0.5);
      } else {
        m_robotDrive.tankDrive(0.0, 0.0);
      }
    }
  }

  @Override
  public void disabledPeriodic() {
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);

  }
}
