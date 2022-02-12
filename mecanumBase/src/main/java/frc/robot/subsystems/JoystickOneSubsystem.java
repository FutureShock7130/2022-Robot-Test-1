// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickOneConstants;

public class JoystickOneSubsystem extends SubsystemBase {
  public Joystick joystickOne;
  public JoystickButton joystickOneButtonA;
  public JoystickButton joystickOneButtonB;
  public JoystickButton joystickOneButtonX;
  public JoystickButton joystickOneButtonY;
  /** Creates a new JoystickOneSubsystme. */
  public JoystickOneSubsystem() {
    joystickOne = new Joystick(JoystickOneConstants.JoystickOnePort);
    joystickOneButtonA = new JoystickButton(joystickOne, JoystickOneConstants.JoystickOneButtonA);
    joystickOneButtonB = new JoystickButton(joystickOne, JoystickOneConstants.JoystickOneButtonB);
    joystickOneButtonX = new JoystickButton(joystickOne, JoystickOneConstants.JoystickOneButtonX);
    joystickOneButtonY = new JoystickButton(joystickOne, JoystickOneConstants.JoystickOneButtonY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
