// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class elevator extends SubsystemBase {
  /** Creates a new lift. */

  TalonFX mLift = new TalonFX(IDConstants.kLiftID);
  DigitalInput wBottomElevator = new DigitalInput(IDConstants.kElevatorBottomSwitch);


  public elevator() {
    mLift.setPosition(0);
    mLift.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorUp() {
    mLift.set(1);
  }

  public void elevatorDown() {
    if (!wBottomElevator.get()) {
      mLift.set(0);
    } else {
    mLift.set(-1);
    }
  }

  public void elevatorStop() {
    mLift.set(0);
  }

  public double liftSpeed() {
    return mLift.get();
  }
}
