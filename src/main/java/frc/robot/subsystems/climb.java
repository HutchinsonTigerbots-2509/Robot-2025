// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class climb extends SubsystemBase {
  /** Creates a new climb. */

  public TalonFX mClimber = new TalonFX(IDConstants.kClimbID);

  public climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void climbUp() {
    mClimber.set(1);
  }


  public void climbDown() {
    mClimber.set(-1);
  }


  public void climbStop() {
    mClimber.set(0);
  }

  public double getClimbSpeed() {
    return mClimber.get();
  }


}
