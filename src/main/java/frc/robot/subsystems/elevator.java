// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;


public class elevator extends SubsystemBase {
  /** Creates a new lift. */

  static public TalonFX mLift = new TalonFX(IDConstants.kLiftID);
  static public DigitalInput wBottomElevator = new DigitalInput(IDConstants.kElevatorBottomSwitch);
  //static public DutyCycleEncoder liftEncoder = new DutyCycleEncoder(8);
  static public Encoder liftEncoder = new Encoder(8, 7);


  public elevator() {
    mLift.setPosition(0);
    mLift.setNeutralMode(NeutralModeValue.Brake);
    liftEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("ElevatorPos", getPos());
    SmartDashboard.putBoolean("LimitSwitch", !wBottomElevator.get());
    if (!wBottomElevator.get()) {
      liftEncoder.reset();
    }
    SmartDashboard.updateValues();
  }

  public void elevatorUp() {
    if (getPos() > 25500) {
      mLift.set(0);
    }
    else {
      mLift.set(1);
    }
  }

  public void elevatorDown() {
    if (!wBottomElevator.get()) {
      mLift.set(0);
      liftEncoder.reset();
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

  public double getPos() {
    return liftEncoder.get();
  }

  public void liftSet(double speed) {
    if (speed < 0 && !wBottomElevator.get()) {
      mLift.set(0);
      liftEncoder.reset();
    }
    else {
      mLift.set(speed);
    }
  }

  public void LiftToPos(double pos) {
    if (getPos() > pos && !(getPos() - pos < 10)) {
      liftSet(-3);
    }
    if (getPos() < pos && !(getPos() - pos < 10)) {
      liftSet(.3);
    }
    if (getPos() - pos < 10) {
      elevatorStop();
    }
  }

}
