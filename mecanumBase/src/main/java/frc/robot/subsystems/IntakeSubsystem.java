// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  TalonSRX intakemotor;
  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakemotor = new TalonSRX(IntakeConstants.IntakeMotor);
    intakemotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intake(){
    intakemotor.set(ControlMode.PercentOutput, IntakeConstants.IntakeSpeed);
  }
  public void output(){
    intakemotor.set(ControlMode.PercentOutput, IntakeConstants.OutputSpeed);
  }
  public void stop(){
    intakemotor.set(ControlMode.PercentOutput, 0);
  }
}
