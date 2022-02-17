// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransporterConstants;

public class TransporterSubsystem extends SubsystemBase {
  
  private final WPI_TalonFX topTransporter = new WPI_TalonFX(TransporterConstants.TopTransporterID);
  private final WPI_TalonFX downTransporter = new WPI_TalonFX(TransporterConstants.DownTransporterID);

  /** Creates a new TransporterSubsystem. */
  public TransporterSubsystem() {
    downTransporter.follow(topTransporter);

    topTransporter.setInverted(false);
    downTransporter.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void transport(){
    topTransporter.set(TransporterConstants.TransportSpeed);
  }

  public void stop(){
    downTransporter.set(0);
  }
}
