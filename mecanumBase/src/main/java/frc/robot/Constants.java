// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final int kFrontLeftMotorID = 1;
    public static final int kRearLeftMotorID = 2;
    public static final int kFrontRightMotorID = 3;
    public static final int kRearRightMotorID = 4;

    public static final boolean kFrontLeftEncoderReversed = false;
    public static final boolean kRearLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kRearRightEncoderReversed = true;

    public static final double kTrackWidth = 0.5682;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.4904;
    // Distance between centers of front and back wheels on robot

    public static final double DriveSpeed = 0;

    public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final int kEncoderCPR =2048 ;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumference = kWheelDiameterMeters * 2 * Math.PI;
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.
    public static final double kS = 0.61194;
    public static final double kV = 0.020779;
    public static final double kA = 0.0011068;

    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
 
    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.5;
    public static final double kPRearLeftVel = 0.5;
    public static final double kPFrontRightVel = 0.5;
    public static final double kPRearRightVel = 0.5;
  }

  public static final class ClimberConstants {
    public static final int RightClimber = 0;
    public static final int LeftClimber = 0;
    public static final int Leftup = 0;
    public static final int Falldown = 0;
  }

  public static final class IntakeConstants {
    public static final int IntakeMotor = 0;
    public static final int IntakeSpeed = 0;
    public static final int OutputSpeed = 0;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class JoystickOneConstants {
    public static final int JoystickOnePort = 0;
    public static final int JoystickOneButtonA = 1;
    public static final int JoystickOneButtonB = 0;
    public static final int JoystickOneButtonX = 0;
    public static final int JoystickOneButtonY = 0;
  }

  public static final class JoystickTwoConstants {
    public static final int JoystickTwoPort = 0;
    public static final int JoystickTwoButtonA = 1;
    public static final int JoystickTwoButtonB = 0;
    public static final int JoystickTwoButtonX = 0;
    public static final int JoystickTwoButtonY = 0;
  }

  public static final class PneumaticsConstants {
    public static final int CompressorModule = 0;
    public static final int IntakeSolenoidModule = 0;
    public static final int IntakeSolenoidForwardChannel = 0;
    public static final int IntakeSolenoidReverseChannel = 0;
    public static final int ClimberSolenoidModule = 0;
    public static final int ClimberSolenoidForwardChannel = 0;
    public static final int ClimberSolenoidReverseChannel = 0;
  }

  public static final class ShooterConstants{
    public static final int SparkMAXmaster = 0;
    public static final int SparkMAXslave = 0;
  }

  public static final class TransportConstants{
    public static final int Transporter = 0;
    public static final double TransportSpeed = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
