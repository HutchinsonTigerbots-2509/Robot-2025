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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class vision extends SubsystemBase {

  DriveSubsystem sDrivetrain;

  PhotonCamera camera1;
  PhotonCamera camera2;
  boolean targetVisible = false;

  public Pose2d aPose2d;

  public double targetYaw = 0.0;
  public double trueX;
  public double trueY;
  public double tagAngle;
  public double trueDistance = 0.0;
  public double posOnScreen = 0;

  HashMap<List, List> targetList = new HashMap<List, List>();
  ArrayList<List> targetIDList = new ArrayList<List>();
  ArrayList<List> targetPOSList = new ArrayList<List>();

  /** Creates a new vision. */
  public vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
  /** Fetch results given from the 2 functional PhotonVision based cameras */
  // @SuppressWarnings({ "unchecked", "rawtypes" })
  public void gainResults() {
    // List<Object> targetList = new ArrayList<Object>();
    // HashMap<List, List> targetList = new HashMap<List, List>();
    // ArrayList<List> targetIDList = new ArrayList<List>();
    // ArrayList<List> targetPOSList = new ArrayList<List>();

    var resultCam1 = camera1.getAllUnreadResults();
    var resultCam2 = camera2.getAllUnreadResults();
    if (!resultCam1.isEmpty()) {
      var result = resultCam1.get(resultCam1.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          // targetIDList.addAll((Collection<? extends List<PhotonTrackedTarget>>) target); TODO: fix stack errors
          // targetPOSList.addAll((Collection<? extends List<Integer>>) result.getTargets());          
        }
      }
    }

    if (!resultCam2.isEmpty()) {
      var result = resultCam2.get(resultCam2.size() - 1);
      if (result.hasTargets()) {   
        for (var target : result.getTargets()) { 
          // targetIDList.addAll((Collection<? extends List<PhotonTrackedTarget>>) target);
          // targetPOSList.addAll((Collection<? extends List<Integer>>) result.getTargets());
        }     
      }
    }

    targetList.put(targetIDList, targetPOSList);
    

    // targetList.put(resultCam2, resultCam2);
    // targetList.put("POS", targetPOSList);

    // targetList.get("ID");
    // targetList.get("POS");

    // TODO: verify the output of these two lists and their listed order.
    
  }

  //** Call this method if you need to split your target IDs and positions/results. */
  public void splitCameraResults() {

  }


  /** Pulls distance using april tag positions last recorded from the field/initialized positions. <p>
   *  Only pulls distance from the first initialized camera. <p>
   * 
   * 
   * references first indexed tag when called.
   * 
   * @return Distance from all vectors using a transform3d, in meters. Specifically getBestCameraToTarget();
   */
  public double fetchTagDistance1() {
    var resultCam1 = camera1.getAllUnreadResults();
    var result = resultCam1.get(resultCam1.size() - 1);
    var target = result.getTargets();
        double trueDistance = (target.get(0)).getBestCameraToTarget().getZ();
    //   }
    // }
    return trueDistance;
  }

  /** Pulls distance using april tag positions last recorded from the field/initialized positions. <p>
   *  Only pulls distance from the first initialized camera. <p>
   * 
   * references first indexed tag when called.
   * 
   * @return Distance from all vectors using a transform3d, in meters. Specifically getBestCameraToTarget();
   */
  public double fetchTagDistance2() {
    var resultCam2 = camera2.getAllUnreadResults();
    var result = resultCam2.get(resultCam2.size() - 1);
    var target = result.getTargets();
    // if (result.hasTargets()) {
    //   for (var target : result.getTargets()) {
        double trueDistance = (target.get(0)).getBestCameraToTarget().getZ();
    //   }
    // }
    return trueDistance;
  }

  public double fetchTagAngle1() {
    var resultCam1 = camera1.getAllUnreadResults();
    var result = resultCam1.get(resultCam1.size() - 1);
    var target = result.getTargets();
    double tagAngle = (target.get(0)).getBestCameraToTarget().getRotation().getAngle();
    return tagAngle;
  }

  public double fetchTagAngle2() {
    var resultCam2 = camera2.getAllUnreadResults();
    var result = resultCam2.get(resultCam2.size() - 1);
    var target = result.getTargets();
    double tagAngle = (target.get(0)).getBestCameraToTarget().getRotation().getAngle();
    return tagAngle;
  }
  
  
  



  //**   Dereks booty land */

  public Pose2d getAprilPos() {

    var resultCam1 = camera1.getAllUnreadResults();
    var result = resultCam1.get(resultCam1.size() - 1);
    var target = result.getBestTarget().getBestCameraToTarget();
    
    aPose2d = new Pose2d(target.getX(), target.getY(), target.getRotation().toRotation2d());

    return aPose2d;
  }



  public double getAprilDistance() {
    var resultCam1 = camera1.getAllUnreadResults();
    var result = resultCam1.get(resultCam1.size() - 1);
    var target = result.getTargets();
        double trueDistanceX = (target.get(0)).getBestCameraToTarget().getMeasureX().in(Meters);
        double trueDistanceY = (target.get(0)).getBestCameraToTarget().getMeasureY().in(Meters);
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
  public double getTagPosOnScreen() {

    var resultCam1 = camera1.getAllUnreadResults();
    var result = resultCam1.get(resultCam1.size() -1);
    var target = result.getBestTarget();


    var targetCorners = target.getDetectedCorners();

    var targetXLeft = Double.valueOf((targetCorners.get(3)).toString());
    var targetXRight = Double.valueOf((targetCorners.get(2)).toString());

    var medianTarget =  (targetXLeft + targetXRight) * .5;

    

    //TODO Calculate posOnScreen
    return medianTarget;
  }

  public double getVisionDrive() {
    double vSpeed = getTagPosOnScreen() * 100;
    if(Math.abs(vSpeed) < 5) {
      vSpeed = 0;
    }
    return vSpeed;
  }

}
