// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class elevatorController extends PIDCommand {
  static final double kP = 0.002;
  // 0.04 before
  static final double kI = 0.001;
  // .003 before
  static final double kD = 0.00;
  /** Creates a new Shoulder. */
  public elevatorController(double desiredPos, elevator sElevator) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        sElevator::getPos,
        // This should return the setpoint (can also be a constant)
        desiredPos,
        // This uses the output
        output -> {
          // Use the output here
          sElevator.liftSet(output);
        });

        

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(sElevator);
    // Configure additional PID options by calling `getController` here.
    this.getController().setTolerance(50);
    this.getController().setSetpoint(desiredPos);
  }

  // Returns true when the command shoutld end.
  @Override
  public boolean isFinished() {

    if (DriverStation.isAutonomous())
      return this.getController().atSetpoint();
    else
      return false;
    
  }
}