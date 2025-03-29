package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class kicker extends SubsystemBase {
  /** Creates a new intake. */

  TalonSRX mKicker = new TalonSRX(IDConstants.kKickerID);


  public kicker() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.updateValues();
  }

  public void kickerSet(double speed) {
    mKicker.set(ControlMode.PercentOutput, speed);
  }

  // public void kickerReverse() {
  //   mKicker.set(ControlMode.PercentOutput, 1);
  // }

  public void kickerStop() {
    mKicker.set(ControlMode.PercentOutput, 0);
  }

}
