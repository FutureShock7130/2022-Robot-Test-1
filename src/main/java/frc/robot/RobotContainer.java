// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverStationPorts;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final ExampleCommand m_autCommand = new ExampleCommand(m_robotDrive);

  // The driver's controller
  Joystick m_driverController = new Joystick(DriverStationPorts.joystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    m_driverController.getRawAxis(DriverStationPorts.leftStick_Y),
                    m_driverController.getRawAxis(DriverStationPorts.leftStick_X),
                    m_driverController.getRawAxis(DriverStationPorts.rightStick_X),
                    false),
                m_robotDrive)
        );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() { 
    // An ExampleCommand will run in autonomous

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics
    );

    // An example trajectory to follow.  All units in meters.
    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config
    );

    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
      m_trajectory, 
      m_robotDrive::getPose, 
      DriveConstants.kFeedforward,
      DriveConstants.kDriveKinematics,

      // xController, yController and thetaController  
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

      // Needed for normalizing wheel speeds
      AutoConstants.kMaxSpeedMetersPerSecond,

      // Velocity PID's
      new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
      new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
      new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
      new PIDController(DriveConstants.kPRearRightVel, 0, 0),

      m_robotDrive::getWheelSpeeds, 
      m_robotDrive::setDriveMotorControllersVolts, 
      m_robotDrive
    );
    
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(m_trajectory.getInitialPose());
    
    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}