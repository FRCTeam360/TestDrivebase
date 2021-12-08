// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;
/** Add your docs here. */
public class XboxArcadeDriveJulia extends CommandBase {
    private final DriveTrain myDriveTrain;

    private final XboxController driverCont;

    public XboxArcadeDriveJulia(DriveTrain driveTrain) {

        driverCont = new XboxController(driverContPort);

        myDriveTrain = driveTrain;

        addRequirements(myDriveTrain);

    }
@Override
public void initialize() {}

@Override
public void execute() {
   
    double upDownSquared; 
    upDownSquared = driverCont.getY(Hand.kLeft)*driverCont.getY(Hand.kLeft);
    double leftRightSquared;
    leftRightSquared = driverCont.getX(Hand.kLeft)*driverCont.getX(Hand.kLeft);
    
    if(driverCont.getY(Hand.kLeft) < 0) {

        upDownSquared = -1*upDownSquared;
    } 
    if(driverCont.getX(Hand.kLeft) < 0) {
        leftRightSquared = -1*leftRightSquared;
    }
    double driveLeft = upDownSquared + leftRightSquared;
    double driveRight = upDownSquared - leftRightSquared;
    driveLeft = Math.min(driveLeft, 1);
    driveLeft = Math.max(driveLeft, -1);
    driveRight = Math.min(driveRight, 1);
    driveRight = Math.max(driveRight, -1);

    myDriveTrain.driveL(driveLeft);
    myDriveTrain.driveR(driveRight);


}
    

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
    return false;
}
}
