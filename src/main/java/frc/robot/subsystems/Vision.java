// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class vision extends SubsystemBase {

  PhotonCamera camera1;
  PhotonCamera camera2;
  boolean targetVisible = false;
  double targetYaw = 0.0;
  double targetRange = 0.0;
  List<Integer> targetList = new ArrayList<>();


  /** Creates a new vision. */
  public vision() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }

  public void gainResults() {
    var resultCam1 = camera1.getAllUnreadResults();
    var resultCam2 = camera2.getAllUnreadResults();
    if (!resultCam1.isEmpty()) {
      var result = resultCam1.get(resultCam1.size() - 1);
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          
        }
      }
    }
    if (!resultCam2.isEmpty()) {
      var result = resultCam2.get(resultCam2.size() - 10);
      if (result.hasTargets()) {   
        for (var target : result.getTargets()) { 
          //* set index to store fetched results per individual target, then loop and combine for cam1 and 2. */
         }     
      }
    }
  }

}
