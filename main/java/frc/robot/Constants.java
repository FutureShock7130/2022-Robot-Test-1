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

    public static final int kEncoderCPR = 2048 ;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;
    public static final double kGearRatio = (50/14) * (48/16);
    public static final double kEncoderDistancePerPulse = kWheelCircumference / (double) kEncoderCPR / kGearRatio;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically for "your" robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double kS = 0.58646;
    public static final double kV = 0.020877;
    public static final double kA = 0.0010482;

    public static final SimpleMotorFeedforward kFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
 
    // Example value only - as above, this must be tuned for your drive!
    public static final double kPFrontLeftVel = 0.43088;
    public static final double kPRearLeftVel = 0.43088;
    public static final double kPFrontRightVel = 0.43088;
    public static final double kPRearRightVel = 0.43088;
  }

  public static final class OIConstants {
    public static final int kDriveTrainJoystickPort = 0;
    public static final int kOthersJoystickPort = 1;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int Btn_A = 1;
    public static final int Btn_B = 2;
    public static final int Btn_X = 3;
    public static final int Btn_Y = 4;
    public static final int Btn_LB = 5;
    public static final int Btn_RB = 6;
    public static final int Btn_LS = 9;  
    public static final int Btn_RS = 10;
  }

  public static final class ShooterConstants{
    public static final int MasterShooterID = 0;
    public static final int SlaveShooterID = 0;

    // 待須測試，之後應該要改成機器人距離籃框某段距離要用多少速度
    public static final double ShootSpeed = 0.15;
  }

  public static final class TransporterConstants{
    public static final int TopTransporterID = 0;
    public static final int DownTransporterID = 0;
    public static final double TransportSpeed = 0.15;
  }

  public static final class ClimberConstants{
    public static final int kFrontLeftMotorID = 0;
    public static final int kRearLeftMotorID = 0;
    public static final int kFrontRightMotorID = 0;
    public static final int kRearRightMotorID = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 10;
    public static final double kMaxAccelerationMetersPerSecondSquared = 7;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.43088;
    public static final double kPYController = 0.43088;
    public static final double kDXYController = 0.050397;
    public static final double kPThetaController = 0.7619;

  // Constraint for the motion profilied robot angle controller
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
