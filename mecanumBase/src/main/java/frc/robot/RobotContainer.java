// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.teleopAimCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.IOException;
import java.nio.file.Path;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveTrainSubsystem m_robotDrive = new DriveTrainSubsystem();

    // The driver's controller
    Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                m_driverController.getRawAxis(0),
                                -m_driverController.getRawAxis(1),
                                m_driverController.getRawAxis(4),
                                false),
                        m_robotDrive));
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
        // Drive at half speed when the RB button is held
        new JoystickButton(m_driverController, 6)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.25))
                .whenReleased(() -> m_robotDrive.setMaxOutput(0.5));
        // rotate and aim at target upon pressing of the A button
        new JoystickButton(m_driverController,1).whenPressed(new teleopAimCommand(m_robotDrive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public void testDrive() {
        m_robotDrive.testMotor();
    }

    public Command getAutonomousCommand() {

        String trajectoryJSON = "paths/testPath.wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            Trajectory testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
                    testTrajectory,
                    m_robotDrive::getPose,
                    DriveConstants.kFeedforward,
                    DriveConstants.kDriveKinematics,

                    // Position contollers
                    new PIDController(AutoConstants.kPXController, 0, 0),
                    new PIDController(AutoConstants.kPYController, 0, 0),
                    new ProfiledPIDController(
                            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

                    // Needed for normalizing wheel speeds
                    AutoConstants.kMaxSpeedMetersPerSecond,

                    // Velocity PID's
                    new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
                    new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
                    new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
                    new PIDController(DriveConstants.kPRearRightVel, 0, 0),
                    m_robotDrive::getCurrentWheelSpeeds,
                    m_robotDrive::setDriveMotorControllersVolts, // Consumer for the output motor voltages
                    m_robotDrive);

            // Reset odometry to the starting pose of the trajectory.
            m_robotDrive.resetOdometry(testTrajectory.getInitialPose());

            // Run path following command, then stop at the end.
            return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
        } catch (IOException e) {
            DriverStation.reportError("Unable to open JSON file", e.getStackTrace());
        }
        return null;

    }
}
