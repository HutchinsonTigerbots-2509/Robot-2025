// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.ControllerMapping;
import frc.robot.subsystems.*;



public class RobotContainer {

    //  🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨   *****     NUMBERS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    public static double MaxSpeed = SwerveConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static double liftPos = 0;
    public static SlewRateLimiter Slewer1 = new SlewRateLimiter(IDConstants.kSlewFilter1);  // Creates our Slew Limiter which makes our drivetrain slowly accelorate, Please Ignore spelling <3
    public static SlewRateLimiter Slewer2 = new SlewRateLimiter(IDConstants.kSlewFilter2);


    // 🚨🚨🚨🚨🚨🚨🚨🚨    *****     BASIC/CONTROLLER INITIALIZATIONS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

    final static CommandXboxController jJoystick = new CommandXboxController(IDConstants.kControllerID);
    final Joystick jButtonBoardPrimary = new Joystick(IDConstants.kPrimaryID);
    final Joystick jButtonBoardSecondary = new Joystick(IDConstants.kSecondaryID);

    
        
    
        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     DRIVE OBJECTS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    
        /* Setting up bindings for necessary control of the swerve drive platform */

        public static double fieldOffset = 0;

        public static boolean visionDriveMode = false;
        public static boolean visionDrive = false;
        public static boolean fieldOriented = true;
        public static double currentVisionPos = IDConstants.kAprilLeftPole;

        public static PIDController drivePID = new PIDController(
            IDConstants.kDriveP, 
            IDConstants.kDriveI, 
            IDConstants.kDriveD, 
            IDConstants.kDriveUpdateRate
            );

        public static final PowerDistribution powerBoard = new PowerDistribution(1, ModuleType.kRev);

        
    
    
    
    
    
        public final static SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
                .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
                
    
    
        public final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        public final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
        //private final Telemetry logger = new Telemetry(MaxSpeed);
        
    
        // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     SUBSYSTEMS     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    
        public static final DriveSubsystem sDrivetrain = SwerveConstants.createDrivetrain();
        public static final climber sClimber = new climber();
        public static final vision sVision = new vision();
        public static final elevator sElevator = new elevator();
        public static final intake sIntake = new intake();
        public static final kicker sKicker = new kicker();
        public static final elastic sElastic = new elastic();
        public static final pathPlannerDrive sPathPlannerDrive = new pathPlannerDrive(sClimber, sDrivetrain, sElevator, sIntake, sKicker, sVision);
    
      
    
    
        public RobotContainer() {
            configureBindings();
        }
    
    
    
        private void configureBindings() {
    
            // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     Objects     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
            RunCommand normDrive = new RunCommand(() -> driveController());
            normDrive.addRequirements(sDrivetrain);
            RunCommand creepDrive = new RunCommand(() -> driveControllerCreep());
            creepDrive.addRequirements(sDrivetrain);
            
    
            // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     Default Commands     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//

            sDrivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                normDrive
                );
    
    
    
    
            // 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨    *****     BUTTON BIND/MAPPING     *****     //🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
    
            JoystickButton ClimbUpBtn;
            ClimbUpBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kClimb);
            ClimbUpBtn.whileTrue(new RunCommand(() -> sClimber.climbUp())).onFalse(new InstantCommand(() -> sClimber.climbStop()));
            ClimbUpBtn.onTrue(new InstantCommand(() -> elastic.selectTab("ClimbTab")));
    
            JoystickButton ClimbDownBtn;
            ClimbDownBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kClimbReverse);
            ClimbDownBtn.whileTrue(new RunCommand(() -> sClimber.climbDown())).onFalse(new InstantCommand(() -> sClimber.climbStop()));
            ClimbDownBtn.onTrue(new InstantCommand(() -> elastic.selectTab("ClimbTab")));
    
            JoystickButton IntakeInBtn;
            IntakeInBtn = new JoystickButton(jButtonBoardPrimary, ControllerMapping.kIntakeIn);
            IntakeInBtn.whileTrue(new RunCommand(() -> sIntake.intakeIn())).onFalse(new InstantCommand(() -> sIntake.intakeStop()));
    
            JoystickButton IntakeOutBtn;
            IntakeOutBtn = new JoystickButton(jButtonBoardPrimary, ControllerMapping.kIntakeOut);
            IntakeOutBtn.whileTrue(new RunCommand(() -> sIntake.intakeOut())).onFalse(new InstantCommand(() -> sIntake.intakeStop()));
            IntakeOutBtn.onTrue(new InstantCommand(() -> elastic.selectTab("TeleTab")));
    
            JoystickButton IntakeLoadBtn;
            IntakeLoadBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kIntakeLoad);
            //IntakeLoadBtn.onTrue(new RunCommand(() -> sIntake.intakeSet(.25)).withTimeout(.25).andThen(new InstantCommand(() -> sIntake.intakeStop())));
            IntakeLoadBtn.onTrue(new RunCommand(() -> sIntake.intakeSet(.25)).until(() -> sIntake.getDistanceTripped()).andThen(new InstantCommand(() -> sIntake.intakeStop())));
    
            JoystickButton LiftUpBtn;
            LiftUpBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorUp);
            LiftUpBtn.whileTrue(new RunCommand(() -> sElevator.elevatorUp())).onFalse(new InstantCommand(() -> sElevator.elevatorStop()));
    
            JoystickButton LiftDownBtn;
            LiftDownBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorDown);
            LiftDownBtn.whileTrue(new RunCommand(() -> sElevator.elevatorDown())).onFalse(new InstantCommand(() -> sElevator.elevatorStop()));
    
    
    
            JoystickButton LiftFloorBtn;
            LiftFloorBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorFloor);
            LiftFloorBtn.onTrue(new elevatorController(IDConstants.kFloorPos, sElevator).until(() -> (LiftUpBtn.getAsBoolean() || LiftDownBtn.getAsBoolean())));
    
            JoystickButton LiftBottomBtn;
            LiftBottomBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorBottom);
            LiftBottomBtn.onTrue(new elevatorController(IDConstants.kBottomPos, sElevator).until(() -> (LiftUpBtn.getAsBoolean() || LiftDownBtn.getAsBoolean())));
    
            JoystickButton LiftLowBtn;
            LiftLowBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorLow);
            LiftLowBtn.onTrue(new elevatorController(IDConstants.kLowPos, sElevator).until(() -> (LiftUpBtn.getAsBoolean() || LiftDownBtn.getAsBoolean())));
    
            JoystickButton LiftMiddleBtn;
            LiftMiddleBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorMiddle);
            LiftMiddleBtn.onTrue(new elevatorController(IDConstants.kMiddlePos, sElevator).until(() -> (LiftUpBtn.getAsBoolean() || LiftDownBtn.getAsBoolean())));
    
            JoystickButton LiftHighBtn;
            LiftHighBtn = new JoystickButton(jButtonBoardSecondary, ControllerMapping.kElevatorHigh);
            LiftHighBtn.onTrue(new elevatorController(IDConstants.kHighPos, sElevator).until(() -> (LiftUpBtn.getAsBoolean() || LiftDownBtn.getAsBoolean())));
    
            JoystickButton ClimbBreakerButton;
            ClimbBreakerButton = new JoystickButton(jButtonBoardPrimary, ControllerMapping.kClimbBreaker);
            ClimbBreakerButton.onTrue(new InstantCommand(() -> reverseSwitchPowerBoard()));
            ClimbBreakerButton.onTrue(new InstantCommand(() -> elastic.selectTab("ClimbTab")));

            JoystickButton KickerBtn;
            KickerBtn = new JoystickButton(jButtonBoardPrimary, ControllerMapping.kKickerUp);
            KickerBtn.whileTrue(new RunCommand(() -> sKicker.kickerSet(.35))).onFalse(new InstantCommand(() -> sKicker.kickerStop()));
    
            JoystickButton KickerReverseBtn;
            KickerReverseBtn = new JoystickButton(jButtonBoardPrimary, ControllerMapping.kKickerDown);
            KickerReverseBtn.whileTrue(new RunCommand(() -> sKicker.kickerSet(-.3))).onFalse(new InstantCommand(() -> sKicker.kickerStop()));

    
            
            jJoystick.b().whileTrue(sDrivetrain.applyRequest(() -> brake));
            //TODO make this actually fix both field2d and gyro
            jJoystick.y().onTrue(new InstantCommand(() -> fieldOffset = 0 - pathPlannerDrive.getField2d().getRobotPose().getRotation().getDegrees()));


            jJoystick.a().onTrue(new InstantCommand(() -> {visionDrive = !visionDrive; //sVision.setCameraDriveMode(visionDriveMode);
                SmartDashboard.putBoolean("VisionMode", visionDrive);}));

            // jJoystick.povLeft().onTrue(new InstantCommand(() -> currentVisionPos = IDConstants.kAprilLeftPole));
            // jJoystick.povRight().onTrue(new InstantCommand(() -> currentVisionPos = IDConstants.kAprilRightPole));
            // jJoystick.povUp().onTrue(new InstantCommand(() -> currentVisionPos = IDConstants.kAprilFeederStation));
            jJoystick.rightBumper().toggleOnTrue(creepDrive);
            jJoystick.rightBumper().onTrue(new InstantCommand(() -> elastic.selectTab("CreepTab")));

        }
    
        public static void driveSwervePathPlanner(ChassisSpeeds speeds) {
            double x = speeds.vxMetersPerSecond;
            double y = speeds.vyMetersPerSecond;
            double z = speeds.omegaRadiansPerSecond;

            if (DriverStation.isAutonomous()) {
            sDrivetrain.applyRequest(() -> 
            drive.withVelocityX(x).withVelocityY(y).withRotationalRate(z)).execute();
            // SmartDashboard.putNumber("txS", drive.VelocityX);
            // SmartDashboard.putNumber("tyS", drive.VelocityY);
            // SmartDashboard.putNumber("tzS", drive.RotationalRate);
            //SmartDashboard.putBoolean("HandDriveDefault", false);
            }
        }

        //** Function that is set as default command for sDrivetrain with fieldOriented changing it on and off */
        public static void driveController() {
            if(fieldOriented) {
                sDrivetrain.applyRequest(() ->  
                    drive.withVelocityX((Slewer1.calculate(calculateFieldX(jJoystick)) * MaxSpeed) * IDConstants.kDriveTrainMultiplier) // Drive forward with negative Y (forward)
                        .withVelocityY((Slewer2.calculate(calculateFieldY(jJoystick)) * MaxSpeed) * IDConstants.kDriveTrainMultiplier) // Drive left with negative X (left)
                        .withRotationalRate(-jJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ).execute();
            }
            else {
                sDrivetrain.applyRequest(() ->  
                    drive.withVelocityX((Slewer1.calculate(-jJoystick.getLeftY()) * MaxSpeed) * IDConstants.kDriveTrainMultiplier) // Drive forward with negative Y (forward)
                        .withVelocityY((Slewer2.calculate(-jJoystick.getLeftX()) * MaxSpeed) * IDConstants.kDriveTrainMultiplier) // Drive left with negative X (left)
                        .withRotationalRate(-jJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                        ).execute();
            }
        }

        public static double calculateFieldX(CommandXboxController controller) {

            double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() + fieldOffset);
            //SmartDashboard.putNumber("gyro", gyro);
            double cos = Math.cos(gyro);
            //SmartDashboard.putNumber("Cos", cos);
            double sin = Math.sin(gyro);
            //SmartDashboard.putNumber("Sin", sin);

            double tX = -controller.getLeftY(); // The X in FRC means forwards and backwards
            double tY = -controller.getLeftX();

            double fieldX = ((tX * cos) + (tY * sin));

            return fieldX;
        }

        public static double calculateFieldY(CommandXboxController controller) {

            double gyro = Math.toRadians(sDrivetrain.getPigeon2().getYaw().getValueAsDouble() + fieldOffset);
            double cos = Math.cos(gyro);
            double sin = Math.sin(gyro);

            double tX = -controller.getLeftY(); // The X in FRC means forwards and backwards
            double tY = -controller.getLeftX();

            double fieldY = ((tY  * cos) - (tX * sin));
            
            //SmartDashboard.putNumber("fieldY", fieldY);

            return fieldY;
        }


        //** Controls the drivetrain while in creep mode with visionDrive deciding if it should auto-orientate on the apriltag */
        public static void driveControllerCreep() {
            
            if(visionDrive) {
                sDrivetrain.applyRequest(() ->
                drive.withVelocityX(-((jJoystick.getLeftY() * MaxSpeed) * IDConstants.kDriveTrainCreep) - 0.3) // Drive forward with negative Y (forward)
                    .withVelocityY(sVision.getMoveVision(currentVisionPos)) // Drive left with negative X (left)
                    .withRotationalRate((-jJoystick.getRightX() * MaxAngularRate) * IDConstants.kDriveTrainCreep * 3)).execute(); // Drive counterclockwise with negative X (left)
            }
            else {
                sDrivetrain.applyRequest(() ->
                drive.withVelocityX((jJoystick.getLeftY() * MaxSpeed) * IDConstants.kDriveTrainCreep - 0.3) // Drive forward with negative Y (forward)
                    .withVelocityY((jJoystick.getLeftX() * MaxSpeed) * IDConstants.kDriveTrainCreep) // Drive left with negative X (left)
                    .withRotationalRate((-jJoystick.getRightX() * MaxAngularRate) * IDConstants.kDriveTrainCreep * 3)).execute(); // Drive counterclockwise with negative X (left)
            }
        }
        


        public void setSwitchablePow(Boolean state) {
            powerBoard.setSwitchableChannel(state);
            SmartDashboard.putBoolean("powerBoardSwitch", !powerBoard.getSwitchableChannel());
        }

        public static void reverseSwitchPowerBoard() {
            powerBoard.setSwitchableChannel(!powerBoard.getSwitchableChannel());
            SmartDashboard.putBoolean("powerBoardSwitch", !powerBoard.getSwitchableChannel());
        }
        
    
        public Command getAutonomousCommand() {
            return pathPlannerDrive.getAutonomousCommand();
        }
        
        

    
    
    public intake getIntake() {
        return sIntake;
    }

    public elevator getElevator() {
        return sElevator;
    }

    public kicker getKicker() {
        return sKicker;
    }

    public DriveSubsystem getDrivetrain() {
        return sDrivetrain;
    }

    public vision getVision() {
        return sVision;
    }

    public climber getClimber() {
        return sClimber;
    }

    public pathPlannerDrive getPathPlannerDrive() {
        return sPathPlannerDrive;
    }

    
}
