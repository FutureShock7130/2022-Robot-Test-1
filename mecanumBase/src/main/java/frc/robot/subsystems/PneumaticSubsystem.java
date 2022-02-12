// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticSubsystem extends SubsystemBase {
  Compressor compressor;
  DoubleSolenoid intakeSolenoid;
  DoubleSolenoid climberSolenoid;
  /** Creates a new PneumaticSubsystem. */
  public PneumaticSubsystem() {
  compressor = new Compressor(PneumaticsConstants.CompressorModule, PneumaticsModuleType.CTREPCM);
  intakeSolenoid = new DoubleSolenoid(PneumaticsConstants.IntakeSolenoidModule, PneumaticsModuleType.CTREPCM, PneumaticsConstants.IntakeSolenoidForwardChannel, PneumaticsConstants.IntakeSolenoidReverseChannel);
  climberSolenoid = new DoubleSolenoid(PneumaticsConstants.ClimberSolenoidModule, PneumaticsModuleType.CTREPCM, PneumaticsConstants.ClimberSolenoidForwardChannel, PneumaticsConstants.ClimberSolenoidReverseChannel);
  }

  @Override
  public void periodic() {
    compressor.enableDigital();
    // This method will be called once per scheduler run
  }
  public void intakereachout(){
    intakeSolenoid.set(Value.kForward);
  }
  public void intakepullback(){
    intakeSolenoid.set(Value.kReverse);
  }
  public void climberreachout(){
    climberSolenoid.set(Value.kForward);
  }
  public void climberpullback(){
    climberSolenoid.set(Value.kReverse);
  }
}
