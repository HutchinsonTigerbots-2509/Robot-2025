// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;

public class shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  //  TalonSRX mCoralShooter = new TalonSRX(IDConstants.kCoralShooterID);
  WPI_TalonSRX mCoralShooter = new WPI_TalonSRX(IDConstants.kCoralShooterID);
  // VictorSPX mCoralShooter = new VictorSPX(IDConstants.kCoralShooterID);
  // WPI_VictorSPX mCoralShooter = new WPI_VictorSPX(IDConstants.kCoralShooterID); 
  // SparkMax mCoralShooter = new SparkMax(IDConstants.kCoralShooterID, MotorType.kBrushed);

  public shooter() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    /**🤯 <p>
     *  Sets the shooter mode to be true
   * 
   * @param Output <p>
   * 
   * insert Output as a number between 1, -1.
   */
  
  public void setShooter(double speed) {
    mCoralShooter.set(speed);
  }


  public Command ShooterSet(double Output) {
    return run(() -> setShooter(Output));
  }

    /** 🤯 <p>
     * Sets the shooter mode to be true
   * 
   * @param Output <p>
   * 
   * insert Output as a number between 1, -1.
     * @return 
   */

  public Command ShooterStop() {
    return run(() -> mCoralShooter.set(0.0));
  }
}
