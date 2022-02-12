// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  TalonFX transporter;
  /** Creates a new Transporter. */
  public TransportSubsystem() {
    transporter = new TalonFX(TransportConstants.Transporter);
    transporter.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void transport(){
    transporter.set(ControlMode.PercentOutput,TransportConstants.TransportSpeed);
  }
  public void stop(){
    transporter.set(ControlMode.PercentOutput, 0);
  }
}
