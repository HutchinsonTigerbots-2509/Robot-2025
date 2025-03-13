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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static double liftPos = 0;
    public static SlewRateLimiter Slewer = new SlewRateLimiter(IDConstants.kSlewFilter);  // Creates our Slew Limiter which makes our drivetrain slowly accelorate, Please Ignore spelling <3



    // 🚨🚨🚨🚨🚨🚨🚨🚨    *****     BASIC/CONTROLLER INITIALIZATIONS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    final static CommandXboxController jJoystick = new CommandXboxController(IDConstants.kControllerID);
    final Joystick jButtonBoardPrimary = new Joystick(IDConstants.kPrimaryID);
    final Joystick jButtonBoardSecondary = new Joystick(IDConstants.kSecondaryID);

    SendableChooser<String> autoSelect = new SendableChooser<>();
    

    // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     DRIVE OBJECTS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    /* Setting up bindings for necessary control of the swerve drive platform */





    //TODO: change .RobotCentric to .FieldCentric for field oriented.... // <---------- 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨




    public final static SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //private final Telemetry logger = new Telemetry(MaxSpeed);


    // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     SUBSYSTEMS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    public final DriveSubsystem sDrivetrain = SwerveConstants.createDrivetrain();
    public final climber sClimb = new climber();
    public final vision sVision = new vision();
    public final elevator sLift = new elevator();
    public final intake sIntake = new intake();
    public final kicker sKicker = new kicker();

    public final Dashboard dashboard = new Dashboard();



    // 🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     START DEFAULT COMMANDS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//



    public RobotContainer() {
        configureBindings();
        namedCommands();
    }

    private void namedCommands() {

        for (double n = .5; n <= 3; n = n + .5) {

        // Actions

            NamedCommands.registerCommand("Intake In " + n, new RunCommand(() -> sIntake.intakeIn()).withTimeout(n));
            NamedCommands.registerCommand("Intake Out " + n, new RunCommand(() -> sIntake.intakeOut()).withTimeout(n));
            NamedCommands.registerCommand("Climb Up " + n, new RunCommand(() -> sClimb.climbUp()).withTimeout(n));
            NamedCommands.registerCommand("Climb Down " + n, new RunCommand(() -> sClimb.climbDown()).withTimeout(n));
            NamedCommands.registerCommand("Lift Up " + n, new RunCommand(() -> sLift.elevatorUp()).withTimeout(n));
            NamedCommands.registerCommand("Lift Down " + n, new RunCommand(() -> sLift.elevatorDown()).withTimeout(n));

        // Preset Poses

            NamedCommands.registerCommand("Floor " + n, new elevatorController(IDConstants.kFloorPos, sLift).withTimeout(n));
            NamedCommands.registerCommand("Bottom " + n, new elevatorController(IDConstants.kBottomPos, sLift).withTimeout(n));
            NamedCommands.registerCommand("Low " + n, new elevatorController(IDConstants.kLowPos, sLift).withTimeout(n));
            NamedCommands.registerCommand("Middle " + n, new elevatorController(IDConstants.kMiddlePos, sLift).withTimeout(n));
            NamedCommands.registerCommand("High " + n, new elevatorController(IDConstants.kHighPos, sLift).withTimeout(n));
        }

        
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
                drive.withVelocityX(Slewer.calculate(-jJoystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(Slewer.calculate(-jJoystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(Slewer.calculate(-jJoystick.getRightX() * MaxAngularRate)) // Drive counterclockwise with negative X (left)
            )
        );


        /*
        jJoystick.a().whileTrue(sDrivetrain.applyRequest(() -> brake));
        jJoystick.b().whileTrue(sDrivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-jJoystick.getLeftY(), -jJoystick.getLeftX()))
        ));
        */
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /*
        jJoystick.back().and(jJoystick.y()).whileTrue(sDrivetrain.sysIdDynamic(Direction.kForward));
        jJoystick.back().and(jJoystick.x()).whileTrue(sDrivetrain.sysIdDynamic(Direction.kReverse));
        jJoystick.start().and(jJoystick.y()).whileTrue(sDrivetrain.sysIdQuasistatic(Direction.kForward));
        jJoystick.start().and(jJoystick.x()).whileTrue(sDrivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        // reset the field-centric heading on left bumper press
                                            //jJoystick.leftBumper().onTrue(sDrivetrain.runOnce(() -> sDrivetrain.seedFieldCentric()));

        //sDrivetrain.registerTelemetry(logger::telemeterize);


        // 🚨🚨🚨🚨🚨🚨🚨    *****     END CTRE INITIALIZED DRIVETRAIN     *****      //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//




        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     BUTTON BIND/MAPPING     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        
        JoystickButton ClimbUpBtn;
        ClimbUpBtn = new JoystickButton(jButtonBoardPrimary, 3);
        ClimbUpBtn.whileTrue(new RunCommand(() -> sClimb.climbUp())).onFalse(new InstantCommand(() -> sClimb.climbStop()));

        JoystickButton ClimbDownBtn;
        ClimbDownBtn = new JoystickButton(jButtonBoardPrimary, 4);
        ClimbDownBtn.whileTrue(new RunCommand(() -> sClimb.climbDown())).onFalse(new InstantCommand(() -> sClimb.climbStop()));

        JoystickButton IntakeInBtn;
        IntakeInBtn = new JoystickButton(jButtonBoardPrimary, 7);
        IntakeInBtn.whileTrue(new RunCommand(() -> sIntake.intakeIn())).onFalse(new InstantCommand(() -> sIntake.intakeStop()));

        JoystickButton IntakeOutBtn;
        IntakeOutBtn = new JoystickButton(jButtonBoardPrimary, 6);
        IntakeOutBtn.whileTrue(new RunCommand(() -> sIntake.intakeOut())).onFalse(new InstantCommand(() -> sIntake.intakeStop()));

        JoystickButton LiftUpBtn;
        LiftUpBtn = new JoystickButton(jButtonBoardPrimary, 1);
        LiftUpBtn.whileTrue(new RunCommand(() -> sLift.elevatorUp())).onFalse(new InstantCommand(() -> sLift.elevatorStop()));

        JoystickButton LiftDownBtn;
        LiftDownBtn = new JoystickButton(jButtonBoardPrimary, 2);
        LiftDownBtn.whileTrue(new RunCommand(() -> sLift.elevatorDown())).onFalse(new InstantCommand(() -> sLift.elevatorStop()));






        JoystickButton LiftFloorBtn;
        LiftFloorBtn = new JoystickButton(jButtonBoardPrimary, 8);
        LiftFloorBtn.onTrue(new elevatorController(IDConstants.kFloorPos, sLift));

        JoystickButton LiftBottomBtn;
        LiftBottomBtn = new JoystickButton(jButtonBoardPrimary, 9);
        LiftBottomBtn.onTrue(new elevatorController(IDConstants.kBottomPos, sLift));

        JoystickButton LiftLowBtn;
        LiftLowBtn = new JoystickButton(jButtonBoardPrimary, 10);
        LiftLowBtn.onTrue(new elevatorController(IDConstants.kLowPos, sLift));

        JoystickButton LiftMiddleBtn;
        LiftMiddleBtn = new JoystickButton(jButtonBoardPrimary, 11);
        LiftMiddleBtn.onTrue(new elevatorController(IDConstants.kMiddlePos, sLift));

        JoystickButton LiftHighBtn;
        LiftHighBtn = new JoystickButton(jButtonBoardPrimary, 12);
        LiftHighBtn.onTrue(new elevatorController(IDConstants.kHighPos, sLift));

        JoystickButton KickerBtn;
        KickerBtn = new JoystickButton(jButtonBoardPrimary, 5);
        KickerBtn.whileTrue(new RunCommand(() -> sKicker.kickerStart())).onFalse(new InstantCommand(() -> sKicker.kickerStop()));




        

        // jJoystick.povDown().toggleOnTrue(sDrivetrain.applyRequest(() ->
        //             drive.withVelocityX(Slewer.calculate(-jJoystick.getLeftY() * MaxSpeed))
        //             .withVelocityY(sVision.getVisionDrive())
        //             .withRotationalRate(Slewer.calculate(-jJoystick.getRightX() * MaxSpeed))
        //     ) 
        // );


        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     AUTONOMOUS PATH CHOOSER     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
        
        //  Middle Autos

            autoSelect.setDefaultOption("M1", "M1");

        //  Audience Autos

            autoSelect.addOption("A1", "A1");
            autoSelect.addOption("A1 Close", "A1C");

        //  Judge Autos

            autoSelect.addOption("J1", "J1");
            autoSelect.addOption("J1 Close", "J1C");


        SmartDashboard.putData(autoSelect);
    }

    public void driveSwerveVision(ChassisSpeeds speeds) {
        sDrivetrain.applyRequest(() -> drive.withVelocityX(speeds.vxMetersPerSecond)
        .withVelocityY(speeds.vyMetersPerSecond)
        .withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto(autoSelect.getSelected());
    }

    public DriveSubsystem getDrivetrain() {
        return sDrivetrain;
    }
    
}
