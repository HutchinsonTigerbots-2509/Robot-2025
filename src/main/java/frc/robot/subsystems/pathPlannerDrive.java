// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.IDConstants;

public class pathPlannerDrive extends SubsystemBase {
  /** Creates a new pathPlannerDrive.*/

  RobotContainer rContainer;
  climber sClimber;
  DriveSubsystem sDrivetrain;
  elevator sElevator;
  intake sIntake;
  kicker sKicker;
  vision sVision;

   Pose2d fixerPose2d;
   Pose2d startPose2d;
   static Field2d field = new Field2d();
   Rotation2d fixerRotation2d;
   Translation2d fixeTranslation2d;
   static RobotConfig config;

   SwerveDrivePoseEstimator eSwerveEstimator;

   public static PIDController rotationController = new PIDController(
   1, 
   0.5, 
   0);

   static SendableChooser<Command> autoSelect;
   
     
    public pathPlannerDrive(climber kClimber, DriveSubsystem kDriveSubsystem,
                            elevator kElevator, intake kIntake, 
                            kicker kKicker, vision kVision) {

      sDrivetrain = kDriveSubsystem;

      sClimber = kClimber;
      sDrivetrain = kDriveSubsystem;
      sElevator = kElevator;
      sIntake = kIntake;
      sKicker = kKicker;
      sVision = kVision;


      sDrivetrain.getPigeon2().reset();
      autoSelect = new SendableChooser<Command>();
   
       
      startPose2d = new Pose2d(0,0, new Rotation2d(0));

      rotationController.setTolerance(2);

      namedCommands();
   

      
      
   
      //     *****     INITIALIZE PATH PLANNER BUILD     *****     //
      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
   
   
         try {
           AutoBuilder.configure(
                  () -> getPose2d(), // Robot pose supplier
                  resetPos2d -> resetPos2d(startPose2d), // Method to reset odometry (will be called if your auto has a starting pose)
                  () -> getChassisSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                  output -> driveChassis(output), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                  new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                          new PIDConstants(5, 0.5, 0.0), // Translation PID constants
                          new PIDConstants(7, 0.5, 0.0) // Rotation PID constants
                  ),
                  RobotConfig.fromGUISettings(), // The robot configuration
                  () -> {
                    
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
   
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                      return false; // alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                  },
                  sDrivetrain // Reference to this subsystem to set requirements
          );
        } catch (IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }

    
    buildAutoChooser();

    SmartDashboard.putData(autoSelect);

    ApplyStart();
   
    
    
    eSwerveEstimator = new SwerveDrivePoseEstimator(
        getKinematics(), getRotation2d(), getModulePositions(), startPose2d
        );
   
   
   
   
   
   
   
   
   
   
   
   
   
     }
   
     @Override
      public void periodic() {
        // This method will be called once per scheduler run
        //SmartDashboard.getData(AutoBuilder.getCurrentPose().);

        //TODO Check if when going between auto and tele this will reset the pose2d,  (Hint we don't want that)

        if (DriverStation.isDisabled()) {
          ApplyStart();
          eSwerveEstimator.resetPose(startPose2d);
        }
        field.setRobotPose(AutoBuilder.getCurrentPose());
        eSwerveEstimator.update(getRotation2d(), getModulePositions());
        SmartDashboard.putData(field);
        SmartDashboard.putData(autoSelect);
        SmartDashboard.putString("RobotPose", AutoBuilder.getCurrentPose().toString());
        SmartDashboard.putNumber("FL", getModulePositions()[0].distanceMeters);
        SmartDashboard.putNumber("FR", getModulePositions()[1].distanceMeters);
        SmartDashboard.putNumber("RL", getModulePositions()[2].distanceMeters);
        SmartDashboard.putNumber("RR", getModulePositions()[3].distanceMeters);
        SmartDashboard.updateValues();
       
     }
   
     //** Returns the List of SwerveModulePositions in ( F:LR  R:LR ) order */
     public SwerveModulePosition[] getModulePositions() {
   
       SwerveModulePosition FL = sDrivetrain.getModule(0).getPosition(false);
       SwerveModulePosition FR = sDrivetrain.getModule(1).getPosition(false);
       SwerveModulePosition RL = sDrivetrain.getModule(2).getPosition(false);
       SwerveModulePosition RR = sDrivetrain.getModule(3).getPosition(false);
   
       SwerveModulePosition[] ModPositions = new SwerveModulePosition[] {
         FL, FR, RL, RR
       };
   
       return ModPositions;
     }
   
     //** Returns the list of SwerveModuleStates in ( F:LR  R:LR ) order */
     public SwerveModuleState[] getModuleStates() {
       SwerveModuleState FL = sDrivetrain.getModule(0).getCurrentState();
       SwerveModuleState FR = sDrivetrain.getModule(1).getCurrentState();
       SwerveModuleState RL = sDrivetrain.getModule(2).getCurrentState();
       SwerveModuleState RR = sDrivetrain.getModule(3).getCurrentState();
   
       SwerveModuleState[] ModStates = new SwerveModuleState[] {
         FL, FR, RL, RR
       };
   
       return ModStates;
     }
   
     //** Returns the kinematics of the swervedrive from drivetrain */
     public SwerveDriveKinematics getKinematics() {
       SwerveDriveKinematics kinematics = sDrivetrain.getKinematics();
       return kinematics;
     }
   
     //** Returns the current Rotation2d of the robot from the pigeon2 */
     public Rotation2d getRotation2d() {
       Rotation2d pos = sDrivetrain.getPigeon2().getRotation2d();
       return pos;
     }
   
     public void setGyro(Double pos) {
       sDrivetrain.getPigeon2().setYaw(pos);
     }
   
     //** Returns the current Position2d of the robots drivetrain using an estimator */
     public Pose2d getPose2d() {
       Pose2d pos2d = eSwerveEstimator.getEstimatedPosition();
       return pos2d;
     }
     
     
     //** Resets the Position2d of the swerve estimator */
     public void resetPos2d(Pose2d pos) {
       sDrivetrain.resetPose(pos);
       setGyro(pos.getRotation().getDegrees());
     }

   
     //** Returns the current ChassisSpeeds of the drivetrain */
     public ChassisSpeeds getChassisSpeeds() {
       ChassisSpeeds chassisSpeeds = sDrivetrain.getKinematics().toChassisSpeeds(getModuleStates());
       return chassisSpeeds;
     }

   
     //** Drives the robot with given speeds */
     public void driveChassis(ChassisSpeeds speeds) {
       RobotContainer.driveSwervePathPlanner(speeds);
     }

   
     public void correctEsti() {
       //TODO make this make correct position of from the seen april tag
       fixerPose2d = new Pose2d(sVision.getX(), sVision.getY(), getRotation2d());
       eSwerveEstimator.resetPose(fixerPose2d);
     }


    public void ApplyStartCool() {
      String autoName = autoSelect.getSelected().toString();
      try {
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
          startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
          //SmartDashboard.putString("Side", "blue");
        }
        else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
          AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).flipPath().mirrorPath().getStartingHolonomicPose().get());
          startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).flipPath().mirrorPath().getStartingHolonomicPose().get();
          //SmartDashboard.putString("Side", "red");
        }
      } catch (IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }

    public void ApplyStart() {
      String autoName = autoSelect.getSelected().getName();
      try {
        AutoBuilder.resetOdom(PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get());
        startPose2d = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingHolonomicPose().get();
      } catch (IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    };

   
     public static Field2d getField2d() {
       return field;
     }
   
    public static Command getAutonomousCommand() {
      // Command auto;
      // if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
      //   auto = AutoBuilder.buildAuto(autoSelect.getSelected().);
      // else
      //   auto = AutoBuilder.buildAuto(autoSelect.getSelected());
      // return auto;
      return autoSelect.getSelected();
  }
  

  /**
     * Defines all named commands for the PathPlanner System, runs as a for loop with timeout n
    */
    public void namedCommands() {
            
        for (double n = .5; n <= 3; n = n + .5) {

        // Actions
    
            NamedCommands.registerCommand("Intake In " + n, new RunCommand(() -> sIntake.intakeIn()).withTimeout(n).andThen(new InstantCommand(() -> sIntake.intakeStop())));
            NamedCommands.registerCommand("Intake Out " + n, new RunCommand(() -> sIntake.intakeOut()).withTimeout(n).andThen(new InstantCommand(() -> sIntake.intakeStop())));
            NamedCommands.registerCommand("Climb Up " + n, new RunCommand(() -> sClimber.climbUp()).withTimeout(n).andThen(new InstantCommand(() -> sClimber.climbStop())));
            NamedCommands.registerCommand("Climb Down " + n, new RunCommand(() -> sClimber.climbDown()).withTimeout(n).andThen(new InstantCommand(() -> sClimber.climbStop())));
            NamedCommands.registerCommand("Lift Up " + n, new RunCommand(() -> sElevator.elevatorUp()).withTimeout(n).andThen(new InstantCommand(() -> sElevator.elevatorStop())));
            NamedCommands.registerCommand("Lift Down " + n, new RunCommand(() -> sElevator.elevatorDown()).withTimeout(n).andThen(new InstantCommand(() -> sElevator.elevatorStop())));

        // Vision Positions

            NamedCommands.registerCommand("VisionR " + n, new RunCommand(() -> driveChassis(new ChassisSpeeds(-.5, sVision.getMoveVision(IDConstants.kAprilRightPole),0))).withTimeout(n));
            NamedCommands.registerCommand("VisionL " + n, new RunCommand(() -> driveChassis(new ChassisSpeeds(-.5, sVision.getMoveVision(IDConstants.kAprilLeftPole),0))).withTimeout(n));
            NamedCommands.registerCommand("VisionF " + n, new RunCommand(() -> driveChassis(new ChassisSpeeds(-.5, sVision.getMoveVision(IDConstants.kAprilFeederStation),0))).withTimeout(n));
    
        // Preset Poses

            NamedCommands.registerCommand("MoveVisionFeeder " + (n + 3), fixMoveAuto(0, 0, (n + 3.0)));
            NamedCommands.registerCommand("MoveVisionLeft " + (n + 3), fixMoveAuto(-.4, 0, (n + 3.0)));
            NamedCommands.registerCommand("MoveVisionRight " + (n + 3), fixMoveAuto(.4, 0, (n + 3.0)));
    
            NamedCommands.registerCommand("Floor " + n, new elevatorController(IDConstants.kFloorPos, sElevator).withTimeout(n));
            NamedCommands.registerCommand("Bottom " + n, new elevatorController(IDConstants.kBottomPos, sElevator).withTimeout(n));
            NamedCommands.registerCommand("Low " + n, new elevatorController(IDConstants.kLowPos, sElevator).withTimeout(n));
            NamedCommands.registerCommand("Middle " + n, new elevatorController(IDConstants.kMiddlePos, sElevator).withTimeout(n));
            NamedCommands.registerCommand("High " + n, new elevatorController(IDConstants.kHighPos, sElevator).withTimeout(n));
        }
    }

    public void buildAutoChooser() {
      autoSelect.setDefaultOption("M1", AutoBuilder.buildAuto("M1"));
      List<String> options = AutoBuilder.getAllAutoNames();
      for (String n : options) {
        autoSelect.addOption(n, AutoBuilder.buildAuto(n));
      };
    }

    public SequentialCommandGroup fixMoveAuto(double kCam, double kRot, double timeout) {
      double desiredPos = kCam;
      double desiredRot = kRot;
      double driveTime = 1;
      double fixerTime = timeout - driveTime;

      return new SequentialCommandGroup(
        new RunCommand(() -> {driveChassis(new ChassisSpeeds(0, sVision.getMoveVision(desiredPos), getRotationMove(desiredRot)));}).withTimeout(fixerTime),
        new RunCommand(() -> {driveChassis(new ChassisSpeeds(-.5, 0, getRotationMove(desiredRot)));}).withTimeout(driveTime)
      );
    }

    public double getRotationMove(double dAngle) {
      double speed;
      double desiredAngle = dAngle;

      speed = rotationController.calculate(getRotation2d().getDegrees(), desiredAngle);

      return speed;
    }

}
