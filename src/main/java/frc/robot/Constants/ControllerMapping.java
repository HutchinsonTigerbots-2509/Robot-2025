// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class ControllerMapping {

    //TODO move buttons over here for clarity?

    // Create joysticks

    public static int kDriverPort = 0;
    public static int kBBPrimaryPort = 1;
    public static int kBBSecondaryPort = 2;

    //public static CommandXboxController pilot = new CommandXboxController(kDriverPort);
    //public static Joystick jButtonBoardPrimary = new Joystick(kBBPrimaryPort);
    //public static Joystick jButtonBoardSecondary = new Joystick(kBBSecondaryPort);

    // Driver Mappings

    public static double action = 0;

    // Co Pilot Mappings

    public static int kAutoLeft = 1;
    public static int kAutoRight = 2;

    public static int kKickerUp = 3;
    public static int kKickerDown = 4;

    public static int kIntakeIn = 5;
    public static int kIntakeOut = 6;

    public static int kCameraSwap = 9;

    // Co Pilot Secondary

    public static int kIntakeLoad = 3;

    public static int kElevatorUp = 4;
    public static int kElevatorDown = 5;

    public static int kClimb = 6;
    public static int kClimbReverse = 7;
    public static int kClimbBreaker = 11; 

    public static int kElevatorFloor = 12;
    public static int kElevatorBottom = 11;
    public static int kElevatorLow = 10;
    public static int kElevatorMiddle = 9;
    public static int kElevatorHigh = 8;

    public static int kAutoLow = 0;
    public static int kAutoMiddle = 0;
    public static int kAutoHigh = 0;
}
