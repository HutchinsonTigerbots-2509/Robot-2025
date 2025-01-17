// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.vision;

public class RobotContainer {
    private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    final CommandXboxController jJoystick = new CommandXboxController(IDConstants.kControllerID);
    final Joystick jButtonBoardPrimary = new Joystick(IDConstants.kPrimaryID);
    final Joystick jButtonBoardSecondary = new Joystick(IDConstants.kSecondaryID);
    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);


    //     *****     SUBSYSTEMS     *****     //

    public final drivetrain sDrivetrain = SwerveConstants.createDrivetrain();
    public final climb sClimb = new climb();
    public final vision sVision = new vision();





    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {

        //  PREMADE -----------------------------



        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        sDrivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            sDrivetrain.applyRequest(() ->
                drive.withVelocityX(-jJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-jJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-jJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        

        jJoystick.a().whileTrue(sDrivetrain.applyRequest(() -> brake));
        jJoystick.b().whileTrue(sDrivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-jJoystick.getLeftY(), -jJoystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        jJoystick.back().and(jJoystick.y()).whileTrue(sDrivetrain.sysIdDynamic(Direction.kForward));
        jJoystick.back().and(jJoystick.x()).whileTrue(sDrivetrain.sysIdDynamic(Direction.kReverse));
        jJoystick.start().and(jJoystick.y()).whileTrue(sDrivetrain.sysIdQuasistatic(Direction.kForward));
        jJoystick.start().and(jJoystick.x()).whileTrue(sDrivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
                                            //jJoystick.leftBumper().onTrue(sDrivetrain.runOnce(() -> sDrivetrain.seedFieldCentric()));

        sDrivetrain.registerTelemetry(logger::telemeterize);


        // END PREMADE ------------------------------

        // Trigger ClimbUpBtn;
        // ClimbUpBtn = new JoystickButton(jButtonBoardPrimary, 5);
        // ClimbUpBtn.whileTrue(new RunCommand(() -> sClimb.climbUp()));

        // Trigger ClimbDownBtn;
        // ClimbDownBtn = new JoystickButton(jButtonBoardPrimary, 6);
        // ClimbDownBtn.whileTrue(new RunCommand(() -> sClimb.climbDown()));

        jJoystick.leftBumper().whileTrue(new RunCommand(() -> sClimb.climbUp()));
        jJoystick.rightBumper().whileTrue(new RunCommand(() -> sClimb.climbDown()));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
