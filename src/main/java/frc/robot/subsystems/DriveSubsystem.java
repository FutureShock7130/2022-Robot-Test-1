// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PWMchannels;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX m_frontLeft = new WPI_TalonFX(PWMchannels.kFrontLeftChannel);
  private final WPI_TalonFX m_rearLeft = new WPI_TalonFX(PWMchannels.kRearLeftChannel);
  private final WPI_TalonFX m_frontRight = new WPI_TalonFX(PWMchannels.kFrontRightChannel);
  private final WPI_TalonFX m_rearRight = new WPI_TalonFX(PWMchannels.kRearRightChannel); 

  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_frontRight);

  // The gyro sensor
  AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, getGyroAngle());

  public Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
      m_frontLeft.getSelectedSensorVelocity() / DriveConstants.wheelGearRatio * 2 * Math.PI * DriveConstants.wheelDiameter / 60,
      m_frontRight.getSelectedSensorVelocity() / DriveConstants.wheelGearRatio * 2 * Math.PI * DriveConstants.wheelDiameter / 60,
      m_rearLeft.getSelectedSensorVelocity() / DriveConstants.wheelGearRatio * 2 * Math.PI * DriveConstants.wheelDiameter / 60,
      m_rearRight.getSelectedSensorVelocity() / DriveConstants.wheelGearRatio * 2 * Math.PI * DriveConstants.wheelDiameter / 60
    );
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, getGyroAngle());
  }

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.setSelectedSensorPosition(0);
    m_frontRight.setSelectedSensorPosition(0);
    m_rearLeft.setSelectedSensorPosition(0);
    m_rearRight.setSelectedSensorPosition(0);
  }
  
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    m_frontRight.setInverted(true);
    m_rearRight.setInverted(true);
    m_frontLeft.setInverted(false);
    m_rearLeft.setInverted(false);
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_drive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else {
      m_drive.driveCartesian(ySpeed, xSpeed, rot);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_odometry.update(getGyroAngle(), getWheelSpeeds());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}