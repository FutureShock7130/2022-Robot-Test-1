// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX leftClimber;
  WPI_TalonFX rightClimber;
  MotorControllerGroup Climber;

  public ClimberSubsystem() {
    leftClimber = new WPI_TalonFX(ClimberConstants.LeftClimber);
    rightClimber = new WPI_TalonFX(ClimberConstants.RightClimber);
    leftClimber.setInverted(true);
    rightClimber.setInverted(true);
    Climber = new MotorControllerGroup(leftClimber, rightClimber);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void leftup(){
    Climber.set(ClimberConstants.Leftup);
  }

  public void putdown(){
    Climber.set(ClimberConstants.Falldown);
  }

  public void stop(){
    Climber.set(0);
  }

}
