// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.server.PathPlannerServer;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final Intake m_intake = new Intake();
      // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final JoystickButton intakeIn = new JoystickButton(m_driverController, XboxController.Button.kX.value);

    private final JoystickButton intakeOut = new JoystickButton(m_driverController, XboxController.Button.kY.value);

    private final PathPlannerTrajectory examplePath;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

                // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        examplePath = PathPlanner.loadPath("ExamplePath", new PathConstraints(3, 2));
        m_robotDrive.resetOdometry(examplePath.getInitialPose());
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> m_robotDrive.arcadeDrive(
                            m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(), -m_driverController.getLeftX()*.6),
                        m_robotDrive));
        m_intake.setDefaultCommand(
            new RunCommand(
                () -> m_intake.stop() , m_intake)
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
        intakeIn.whileTrue(new IntakeIn(m_intake));
        intakeOut.whileTrue(new IntakeOut(m_intake));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // This trajectory can then be passed to a path follower such as a
        // PPRamseteCommand


        // Run path following command, then stop at the end.
        return new IntakeOut(m_intake).withTimeout(2).andThen(m_robotDrive.followTrajectoryCommand(examplePath, true).andThen(() -> m_robotDrive.tankDriveVolts(0, 0)));
    }

    public void autonomousInit(){
        PathPlannerServer.startServer(5811);
     //   m_robotDrive.resetEncoders();
    }

    public void teleopInit(){
        m_robotDrive.resetEncoders();
    }
}