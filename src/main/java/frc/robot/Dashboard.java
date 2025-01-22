// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/** Add your docs here. */
public class Dashboard {

    public final drivetrain sDrivetrain = SwerveConstants.createDrivetrain();
    public final climb sClimb = new climb();
    public final vision sVision = new vision();
    public final lift sLift = new lift();
    public final intake sIntake = new intake();
    public final grabber sGrabber = new grabber();

    public final IDConstants idConstants = new IDConstants();
    public final SwerveConstants swerveConstants = new SwerveConstants();

    public Dashboard() {
        SendDashboard();
    }

    public void SendDashboard() {
        
    }

}
