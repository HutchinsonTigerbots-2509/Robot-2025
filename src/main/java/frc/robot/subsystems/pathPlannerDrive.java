// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class pathPlannerDrive extends SubsystemBase {
  /** Creates a new pathPlannerDrive. 
   * @throws ParseException 
   * @throws IOException */


  
  public pathPlannerDrive(drivetrain sDrivetrain) throws IOException, ParseException {

    Pose2d startPose2d = new Pose2d(0, 0, getRotation2d(sDrivetrain));

    SwerveDrivePoseEstimator eSwerveEstimator = new SwerveDrivePoseEstimator(
      getKinematics(sDrivetrain), getRotation2d(sDrivetrain), getModulePositions(sDrivetrain), startPose2d
      );








    //        BUILDER :LASDJFLKJAL:KFJALK:DJFLK:AJDF:LJADF //





    AutoBuilder.configure(
            () -> getPose2d(eSwerveEstimator), // Robot pose supplier
            resetPos2d -> resetPos2d(eSwerveEstimator, startPose2d), // Method to reset odometry (will be called if your auto has a starting pose)
            () -> getChassisSpeeds(sDrivetrain), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            driveChassis -> driveChassis(sDrivetrain), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            RobotConfig.fromGUISettings(), // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            sDrivetrain // Reference to this subsystem to set requirements
    );
















  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public SwerveModulePosition[] getModulePositions(drivetrain sDrivetrain) {

    SwerveModulePosition FL = sDrivetrain.getModule(0).getPosition(false);
    SwerveModulePosition FR = sDrivetrain.getModule(1).getPosition(false);
    SwerveModulePosition RL = sDrivetrain.getModule(2).getPosition(false);
    SwerveModulePosition RR = sDrivetrain.getModule(3).getPosition(false);

    SwerveModulePosition[] ModPositions = new SwerveModulePosition[] {
      FL, FR, RL, RR
    };

    return ModPositions;
  }

  public SwerveModuleState[] getModuleStates(drivetrain sDrivetrain) {
    SwerveModuleState FL = sDrivetrain.getModule(0).getCurrentState();
    SwerveModuleState FR = sDrivetrain.getModule(1).getCurrentState();
    SwerveModuleState RL = sDrivetrain.getModule(2).getCurrentState();
    SwerveModuleState RR = sDrivetrain.getModule(3).getCurrentState();

    SwerveModuleState[] ModStates = new SwerveModuleState[] {
      FL, FR, RL, RR
    };

    return ModStates;
  }

  public SwerveDriveKinematics getKinematics(drivetrain sDrivetrain) {
    SwerveDriveKinematics kinematics = sDrivetrain.getKinematics();
    return kinematics;
  }

  public Rotation2d getRotation2d(drivetrain sDrivetrain) {
    Rotation2d pos = sDrivetrain.getPigeon2().getRotation2d();
    return pos;
  }

  public Pose2d getPose2d(SwerveDrivePoseEstimator estimator) {
    Pose2d pos2d = estimator.getEstimatedPosition();
    return pos2d;
  }
  
  public void resetPos2d(SwerveDrivePoseEstimator estimator, Pose2d pos) {
    estimator.resetPose(pos);
  }

  public ChassisSpeeds getChassisSpeeds(drivetrain sDrivetrain) {
    ChassisSpeeds chassisSpeeds = sDrivetrain.getKinematics().toChassisSpeeds(getModuleStates(sDrivetrain));
    return chassisSpeeds;
  }

  public void driveChassis(drivetrain sDrivetrain) {}

}
