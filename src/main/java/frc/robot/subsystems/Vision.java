// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class vision extends SubsystemBase {

  DriveSubsystem sDrivetrain;

  public static PhotonCamera camera1 = new PhotonCamera(IDConstants.kCamera1);
  //public static PhotonCamera camera2 = new PhotonCamera(IDConstants.kCamera2);
  //public static PhotonCamera camera3 = new PhotonCamera(IDConstants.kCamera3);
  public static boolean targetVisible = false;

  public static Pose2d aPose2d;

  public static double targetYaw = 0.0;
  public static double trueX;
  public static double trueY;
  public static double tagAngle;
  public static double trueDistance = 0.0;
  public static double posOnScreen = 0;
  public static double posOnScreenOffset = IDConstants.kCamera1Res / 2;
  
  public static double aprilParallax = .2;
  public static PIDController visionDrivePID;
  public static double kP = 1.5;
  public static double kI = 0.5;
  public static double kD = 0;
  
  public static List<PhotonPipelineResult> resultCam1;
  public static PhotonPipelineResult result;
  public static PhotonTrackedTarget target;
  public static PhotonTrackedTarget defaultTarget;
  public static List<TargetCorner> targetCorners;

  /** Creates a new vision. */
  public vision() {
    
    defaultTarget = new PhotonTrackedTarget();
    target = defaultTarget;
    visionDrivePID = new PIDController(
      kP, 
      kI, 
      kD);
    visionDrivePID.setTolerance(.005);
    //camera1.setDriverMode(false);
    //camera2.setDriverMode(false);
  }

  @Override
  public void periodic() {
    gainResults();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Screen", getTagPosOnScreen());
    SmartDashboard.putNumber("tag Angle", fetchTagAngle());
    SmartDashboard.putBoolean("SeeTag", getSeeTag());
    SmartDashboard.putNumber("AprilNum", getTargetNumber());
    SmartDashboard.putNumber("AprilAngle", getVisionAngle());
    SmartDashboard.putNumber("AprilHeight", getTagHighScreen());
    SmartDashboard.updateValues();
  }


  /** Fetch results given from the 2 functional PhotonVision based cameras */
  // @SuppressWarnings({ "unchecked", "rawtypes" })
  public void gainResults() {

    resultCam1 = camera1.getAllUnreadResults();

    if (!resultCam1.isEmpty()) {
      result = resultCam1.get(resultCam1.size() - 1);
      if (result.hasTargets()) {
        target = result.getBestTarget();
      }
    } 
    else {
      resultCam1.clear();
    }
  }

  public static Boolean getSeeTag() {
    Boolean seeTag;
    seeTag = targetVisible;
    return seeTag;
  }


  /** Pulls distance using april tag positions last recorded from the field/initialized positions. <p>
   *  Only pulls distance from the first initialized camera. <p>
   * 
   * 
   * references first indexed tag when called.
   * 
   * @return Distance from all vectors using a transform3d, in meters. Specifically getBestCameraToTarget();
   */
  public double fetchTagDistance() {
    double trueDistance = target.getBestCameraToTarget().getX();
    if (trueDistance == defaultTarget.getBestCameraToTarget().getX())
      return 0;
    return trueDistance;
  }

  public double fetchTagAngle() {
    double tagAngle = target.getPitch();
    if (tagAngle == defaultTarget.getPitch())
      return 0;
    return tagAngle;
  }
  

  //**   Dereks booty land */

  public Pose2d getAprilPos() {

    //aPose2d = new Pose2d(target.getX(), target.getY(), target.getRotation().toRotation2d());

    return null;
  }



  public double getAprilDistance() {
    double trueDistanceX = target.getBestCameraToTarget().getMeasureX().in(Meters);
    double trueDistanceY = target.getBestCameraToTarget().getMeasureY().in(Meters);
    double DistanceToTarget =  Math.pow(.5,(trueDistanceX * trueDistanceX) + (trueDistanceY * trueDistanceY));
    return DistanceToTarget;
  }




  public double getX() {
    trueX = (getAprilPos().getX() - (getAprilPos().getRotation().getSin() * getAprilDistance()));
    return trueX;
  }

  public double getY() {
    trueY = (getAprilPos().getY() - (getAprilPos().getRotation().getCos() * getAprilDistance()));
    return trueY;
  }

  /**
   * Calculates the pos of the tag on screen with a middle of 0 <p>
   *
   * @return double from -1 to 1
   */
  public static double getTagPosOnScreen() {

    double targetXLeft;
    double targetXRight;

    targetCorners = target.getDetectedCorners();

    if (targetCorners != defaultTarget.getDetectedCorners()) {

      targetXLeft = targetCorners.get(3).x;
      targetXRight = targetCorners.get(2).x;
    }
    else {
      targetXLeft = posOnScreenOffset;
      targetXRight = posOnScreenOffset;
    }

    

    double medianTarget =  (targetXLeft + targetXRight) * .5;

    double pos = (medianTarget - posOnScreenOffset) / posOnScreenOffset;

    return pos;
  }



  public static double getTagHighScreen() {

    double pitch = target.getPitch();


    double pos = (pitch + 14) / 28;  // camera returned pitch of -14 - 14

    return pos;
  }

  public Pose2d getPose2d() {
    Pose2d pos;
    pos = new Pose2d(0, 0, null);
    return pos;
  }

  public double getMoveVision(double desiredPos) {
    SmartDashboard.putNumber("visionDriveSpeed", visionDrivePID.calculate(getTagPosOnScreen(), desiredPos));
    if (visionDrivePID.atSetpoint())
      return 0;
    return -visionDrivePID.calculate(getTagPosOnScreen(), desiredPos + calculateParallax());
  }

  public double calculateParallax() {
    return (aprilParallax * getTagHighScreen());
  }

  public void setCameraDriveMode(Boolean mode) {
    camera1.setDriverMode(mode);
    //camera2.setDriverMode(mode);
  }

  public int getTargetNumber() {
    return target.getFiducialId();
  }

  public double getVisionAngle() {
    int tNum = getTargetNumber();
    double dAngle;
    if (tNum == 11 || tNum == 20) {
      dAngle = 60;
    }
    else if (tNum == 10 || tNum == 21) {
      dAngle = 0;
    }
    else if (tNum == 9 || tNum == 22) {
      dAngle = -60;
    }
    else if (tNum == 8 || tNum == 17) {
      dAngle = -120;
    }
    else if (tNum == 7 || tNum == 18) {
      dAngle = 180;
    }
    else if (tNum == 6 || tNum == 19) {
      dAngle = 120;
    }
    else {
      dAngle = 0;
    }
    return dAngle;
  }

}
