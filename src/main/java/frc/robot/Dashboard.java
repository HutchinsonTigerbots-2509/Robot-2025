// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/** Add your docs here. */
public class Dashboard {

    //public final DriveSubsystem sDrivetrain = SwerveConstants.createDrivetrain();
    public final climber sClimb = new climber();
    public final vision sVision = new vision();
    public final elevator sLift = new elevator();
    public final intake sIntake = new intake();
    public final kicker sKicker = new kicker();

    public final IDConstants idConstants = new IDConstants();
    public final SwerveConstants swerveConstants = new SwerveConstants();

    public Dashboard() {
        SendDashboard();
    }

    public void SendDashboard() {
        
        
    }

}
