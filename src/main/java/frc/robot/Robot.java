// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;

import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private DriveSubsystem m_driveSubsystem;

  private DriveSubsystem Ddrivetrain = SwerveConstants.createDrivetrain();


  

  public Robot() {
    //** Subsystems */
    m_robotContainer = new RobotContainer();
    m_robotContainer.getDrivetrain();

    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

        // m_driveSubsystem.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     m_driveSubsystem.applyRequest(() ->
        //         RobotContainer.drive.withVelocityX(-RobotContainer.jJoystick.getLeftY() * RobotContainer.MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-RobotContainer.jJoystick.getLeftX() * RobotContainer.MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-RobotContainer.jJoystick.getRightX() * RobotContainer.MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );    
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
