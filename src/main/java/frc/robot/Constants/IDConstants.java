// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class IDConstants {


    //     *****     MISC NUM     *****     //

    public static final double kDriveTrainMultiplier = .50;
    public static final double kDriveTrainCreep = .15;

    public static final boolean kFieldOriented = false;

    // Drive PID #s
    public static final double kDriveP = 1;
    public static final double kDriveI = 0;
    public static final double kDriveD = 0;
    public static final double kDriveUpdateRate = 40;

    // April Tag PosesOnScreen for different drops
    public static final double kAprilFeederStation = 0;
    public static final double kAprilRightPole = .6;
    public static final double kAprilLeftPole = -.6;

    //     *****     JOYSTICKS     *****     //

    public static final int kControllerID = 0;
    public static final int kPrimaryID = 1;
    public static final int kSecondaryID = 2;

    public static final double kSlewFilter1 = 2.0;
    public static final double kSlewFilter2 = 2.5;


    //     *****     INTAKE/LIFT/KICKER     *****     //

    public static final int kLiftID = 50;

    public static final double kFloorPos = 0;
    public static final double kBottomPos = 6500;
    public static final double kLowPos = 10700;
    public static final double kMiddlePos = 16600;
    public static final double kHighPos = 25500;

    public static final int kLiftSensorID = 21;
    public static final int kDistanceSensorID = 6;

    public static final int kCoralIntakeID = 51;
    public static final int kCoralSensorID = 23;
    
    public static final int kAlgaeIntakeID = 24;

    public static final int kCoralShooterID = 51;

    public static final int kKickerID = 54;


    //     *****     CLIMB     *****     //

    public static final int kClimbID = 55;

    //     *****     VISION      *****     //
    public static final String kCamera1 = "CameraOut";
    public static final String kCamera2 = "CameraIn";
    public static final double kCamera1Res = 864;
    public static final double kCamera2Res = 900;

    //     *****     LIMIT SWITCHES      *****     //
    public static final int kElevatorBottomSwitch = 9;




}