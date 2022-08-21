// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.deser.std.FromStringDeserializer;

// import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Victor frontRight = new Victor(0);
  private Victor frontLeft = new Victor(1);
  private Victor rearRight = new Victor(3);
  private Victor rearLeft = new Victor(2);
  private Victor conveyor = new Victor(4);

  private MotorControllerGroup rightGroup = new MotorControllerGroup(frontRight, rearRight);
  private MotorControllerGroup leftGroup = new MotorControllerGroup(frontLeft, rearLeft);
  // private AHRS ahrs = new AHRS(SPI.Port.kMXP);
  private XboxController controller = new XboxController(1);
  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  private Timer autoTimer = new Timer();
  {

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    leftGroup.setInverted(false);
    rightGroup.setInverted(true);
    // ahrs.reset();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimer.reset();
    autoTimer.start();
    // ahrs.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // System.out.println("Gyro Angle: " + ahrs.getAngle());
    // if(autoTimer.get() < 2) {
    //   if(ahrs.getAngle() > 0.0){
    //     drive.tankDrive(0.5,0.7);
    //   } else if (ahrs.getAngle()<0.0) {
    //     drive.tankDrive(0.7,0.5);
    //   } else {
    //     drive.tankDrive(0.5,0.5);
    //   }
    // } else if (autoTimer.get() <2.7){
    //   drive.tankDrive(-0.5,0.5);
    // } else if(autoTimer.get() < 4.7) {
    //     if(ahrs.getAngle() > -90.0){
    //       drive.tankDrive(0.5,0.7);
    //     } else if (ahrs.getAngle()< -90.0) {
    //       drive.tankDrive(0.7,0.5);
    //     } else {
    //       drive.tankDrive(0.5,0.5);
    //     }
    //   } else if (autoTimer.get() < 5.4) {
    //     drive.tankDrive(-0.5,0.5);
        
    //   }  else if(autoTimer.get() < 7.4) {
    //       if(ahrs.getAngle() > -180.0){
    //         drive.tankDrive(0.5,0.7);
    //       } else if (ahrs.getAngle()< -180.0) {
    //         drive.tankDrive(0.7,0.5);
    //       } else {
    //         drive.tankDrive(0.5,0.5);
    //       }
    //     } else if (autoTimer.get() < 8.1) {
    //       drive.tankDrive(-0.5,0.5);
          
    //     }  else if(autoTimer.get() < 10.1) {
    //         if(ahrs.getAngle() > -270.0){
    //           drive.tankDrive(0.5,0.7);
    //         } else if (ahrs.getAngle()<-270.0) {
    //           drive.tankDrive(0.7,0.5);
    //         } else {
    //           drive.tankDrive(0.5,0.5);
    //         }
    //  //} else if(autoTimer.get() > 2.5 < 3) {
    //  // drive.arcadeDrive(.5,1);
    // } else {
    //   drive.tankDrive(0,0);
    // }
  }
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   
    drive.arcadeDrive(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller.getLeftX());

    if(controller.getAButton())
    {
      conveyor.set(0.5);
    }
    else if(controller.getBButton())
    {
      conveyor.set(-0.5);
    }
    else{
      conveyor.set(0);
    }
    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
