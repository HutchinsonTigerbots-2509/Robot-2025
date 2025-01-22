// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class grabber extends SubsystemBase {
  /** Creates a new grabber. */

  public TalonFX mGrabber = new TalonFX(IDConstants.kCoralGrabberID);
  
  public grabber() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}