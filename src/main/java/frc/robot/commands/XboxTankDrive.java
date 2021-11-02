// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;
/** Add your docs here. */
public class XboxTankDrive extends CommandBase {
    private final DriveTrain myDriveTrain;

    private final XboxController driverCont;

    public XboxTankDrive(DriveTrain driveTrain) {

        driverCont = new XboxController(driverContPort);
        
        myDriveTrain = driveTrain;

        addRequirements(myDriveTrain);

        }

@Override
public void initialize() {}

@Override
public void execute() {
    if(Math.abs(driverCont.getY(Hand.kRight)) >= xboxDeadzone ) {
        myDriveTrain.driveR((-1 * driverCont.getY(Hand.kRight))); 
    } else if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone ) {
        myDriveTrain.driveL((-1 * driverCont.getY(Hand.kLeft)));
    } else if(Math.abs(driverCont.getY(Hand.kRight)) < xboxDeadzone ) {
        myDriveTrain.driveR((0 * driverCont.getY(Hand.kRight)));
    } else if(Math.abs(driverCont.getY(Hand.kLeft)) < xboxDeadzone ) {
        myDriveTrain.driveL((0 * driverCont.getY(Hand.kLeft)));
    }
   
}

@Override
public void end(boolean interrupted) {}

@Override
public boolean isFinished() {
    return false;
}
}