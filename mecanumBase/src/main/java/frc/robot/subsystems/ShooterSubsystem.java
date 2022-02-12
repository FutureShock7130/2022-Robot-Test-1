// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  CANSparkMax masterShooter;
  CANSparkMax slaveShooter;
  public ShooterSubsystem() {
    masterShooter = new CANSparkMax(ShooterConstants.SparkMAXmaster, MotorType.kBrushless); 
    slaveShooter = new CANSparkMax(ShooterConstants.SparkMAXslave, MotorType.kBrushless); 
    slaveShooter.follow(masterShooter, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(){
    masterShooter.set(0.15);
  }

  public void stop(){
    masterShooter.set(0);
  }
}
