// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.ScheduledThreadPoolExecutor;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;
import frc.robot.Dashboard;

public class RobotContainer {

    //  🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨   *****     NUMBERS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    private double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity



    // 🚨🚨🚨🚨🚨🚨🚨🚨    *****     BASIC/CONTROLLER INITIALIZATIONS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    final CommandXboxController jJoystick = new CommandXboxController(IDConstants.kControllerID);
    final Joystick jButtonBoardPrimary = new Joystick(IDConstants.kPrimaryID);
    final Joystick jButtonBoardSecondary = new Joystick(IDConstants.kSecondaryID);

    SendableChooser<String> autoSelect = new SendableChooser<>();
    

    // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     DRIVE OBJECTS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //private final Telemetry logger = new Telemetry(MaxSpeed);


    // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     SUBSYSTEMS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    public final drivetrain sDrivetrain = SwerveConstants.createDrivetrain();
    public final climber sClimb = new climber();
    public final vision sVision = new vision();
    public final elevator sLift = new elevator();
    public final intake sIntake = new intake();
    public final grabber sGrabber = new grabber();
    public final shooter sShooter = new shooter();

    public final Dashboard dashboard = new Dashboard();



    // 🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     START DEFAULT COMMANDS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//



    public RobotContainer() {
        configureBindings();
        namedCommands();
    }

    private void namedCommands() {

        for (int n = 1; n <= 5; n++) {

        NamedCommands.registerCommand("Intake In" + n, new RunCommand(() -> sIntake.intakeIn()).withTimeout(n));
        NamedCommands.registerCommand("Intake Out" + n, new RunCommand(() -> sIntake.intakeOut()).withTimeout(n));
        NamedCommands.registerCommand("Climb Up" + n, new RunCommand(() -> sClimb.climbUp()).withTimeout(n));
        NamedCommands.registerCommand("Climb Down" + n, new RunCommand(() -> sClimb.climbDown()).withTimeout(n));
        NamedCommands.registerCommand("Lift Up" + n, new RunCommand(() -> sLift.elevatorUp()).withTimeout(n));
        NamedCommands.registerCommand("Lift Down" + n, new RunCommand(() -> sLift.elevatorDown()).withTimeout(n));
        }

        NamedCommands.registerCommand("Grab", new InstantCommand(() -> sGrabber.close()));
        NamedCommands.registerCommand("Release", new InstantCommand(() -> sGrabber.open()));

        





    }


    private void configureBindings() {

        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     END DEFAULT COMMANDS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//




        // 🚨🚨🚨🚨🚨🚨🚨🚨    *****     START CTRE INITIALIZED DRIVETRAIN     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//



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

        //sDrivetrain.registerTelemetry(logger::telemeterize);


        // 🚨🚨🚨🚨🚨🚨🚨    *****     END CTRE INITIALIZED DRIVETRAIN     *****      //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//




        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     BUTTON BIND/MAPPING     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        // Trigger ClimbUpBtn;
        // ClimbUpBtn = new JoystickButton(jButtonBoardPrimary, 5);
        // ClimbUpBtn.whileTrue(new RunCommand(() -> sClimb.climbUp()));

        // Trigger ClimbDownBtn;
        // ClimbDownBtn = new JoystickButton(jButtonBoardPrimary, 6);
        // ClimbDownBtn.whileTrue(new RunCommand(() -> sClimb.climbDown()));

        jJoystick.leftBumper().whileTrue(new RunCommand(() -> sClimb.climbUp())).onFalse(new InstantCommand(() -> sClimb.climbStop()));
        jJoystick.rightBumper().whileTrue(new RunCommand(() -> sClimb.climbDown())).onFalse(new InstantCommand(() -> sClimb.climbStop()));

        // jJoystick.leftBumper().whileTrue(new RunCommand(() -> sLift.elevatorUp())).onFalse(new InstantCommand(() -> sLift.elevatorStop()));
        // jJoystick.rightBumper().whileTrue(new RunCommand(() -> sLift.elevatorDown())).onFalse(new InstantCommand(() -> sLift.elevatorStop()));

        // jJoystick.rightBumper().whileTrue(sShooter.ShooterSet(.5)).onFalse(sShooter.ShooterStop());
        // jJoystick.leftBumper().whileTrue(sShooter.ShooterSet(-.5)).onFalse(sShooter.ShooterStop());


        JoystickButton ShooterOutBtn;
        // ShooterOutBtn = new JoystickButton(jButtonBoardPrimary, 1);
        // ShooterOutBtn.whileTrue(sShooter.ShooterSet(.5)).onFalse(sShooter.ShooterStop());

        JoystickButton ShooterInBtn;
        // ShooterInBtn = new JoystickButton(jButtonBoardPrimary, 2);
        // ShooterInBtn.whileTrue(sShooter.ShooterSet(-.5)).onFalse(sShooter.ShooterStop());
        

        // TODO MAKE DROP OF VISION MODE
        jJoystick.povDown().toggleOnTrue(sDrivetrain.applyRequest(() ->
                    drive.withVelocityX(-jJoystick.getLeftY() * MaxSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(-jJoystick.getRightX() * MaxSpeed)
            ) 
        );


        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     AUTONOMOUS PATH CHOOSER     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        
        autoSelect.setDefaultOption("Middle", "M1");
        autoSelect.addOption("Audience", "A1");
        autoSelect.addOption("Judge", "J1");

        SmartDashboard.putData(autoSelect);

    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto(autoSelect.getSelected());
    }
    
}
