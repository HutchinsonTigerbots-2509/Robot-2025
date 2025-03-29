// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class climber extends SubsystemBase {
  /** Creates a new climb. */

  public WPI_TalonSRX mClimber = new WPI_TalonSRX(IDConstants.kClimbID);

  public climber() {
    mClimber.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.updateValues();
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
