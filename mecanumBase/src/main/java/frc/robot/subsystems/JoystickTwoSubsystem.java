// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickTwoConstants;

public class JoystickTwoSubsystem extends SubsystemBase {
  public Joystick joystickTwo;
  public JoystickButton joystickTwoButtonA;
  public JoystickButton joystickTwoButtonB;
  public JoystickButton joystickTwoButtonX;
  public JoystickButton joystickTwoButtonY;
  /** Creates a new JoystickTwoSubsystem. */
  public JoystickTwoSubsystem() {
    joystickTwo = new Joystick(JoystickTwoConstants.JoystickTwoPort);
    joystickTwoButtonA = new JoystickButton(joystickTwo, JoystickTwoConstants.JoystickTwoButtonA);
    joystickTwoButtonB = new JoystickButton(joystickTwo, JoystickTwoConstants.JoystickTwoButtonB);
    joystickTwoButtonX = new JoystickButton(joystickTwo, JoystickTwoConstants.JoystickTwoButtonX);
    joystickTwoButtonY = new JoystickButton(joystickTwo, JoystickTwoConstants.JoystickTwoButtonY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
