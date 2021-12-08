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

        double upDownSquared; 
        upDownSquared = 0;

    }
@Override
public void initialize() {}

@Override
public void execute() {
    if( driverCont.getY(Hand.kLeft) < 0) {
        myDriveTrain.driveR(-1*(driverCont.getY(Hand.kLeft)*(driverCont.getY(Hand.kLeft))+(driverCont.getX(Hand.kLeft))));
    myDriveTrain.driveL(-1*(driverCont.getY(Hand.kLeft)*(driverCont.getY(Hand.kLeft))+(driverCont.getX(Hand.kLeft))));
    } else {
        myDriveTrain.driveR(driverCont.getY(Hand.kLeft)*(driverCont.getY(Hand.kLeft)+(driverCont.getX(Hand.kLeft))));
    myDriveTrain.driveL(driverCont.getY(Hand.kLeft)*(driverCont.getY(Hand.kLeft)+(driverCont.getX(Hand.kLeft))));
    }

}
    

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
    return false;
}
}
