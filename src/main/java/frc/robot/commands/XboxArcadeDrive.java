// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class XboxArcadeDrive extends CommandBase {

  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  /** Creates a new XboxArcadeDrive. */
  public XboxArcadeDrive(DriveTrain driveTrain) {
    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightLeftSquared = driverCont.getX(Hand.kLeft)*driverCont.getX(Hand.kLeft);
    double upDownSquared = driverCont.getY(Hand.kLeft)*driverCont.getY(Hand.kLeft);
    
    if(driverCont.getX(Hand.kLeft) < 0){
        rightLeftSquared = rightLeftSquared*(-1);
      }
    
    if(driverCont.getY(Hand.kLeft) < 0){
        upDownSquared = upDownSquared*(-1);
      }
    double L = upDownSquared - rightLeftSquared;
    double R = upDownSquared + rightLeftSquared;
     
    Math.max(upDownSquared, -1);
    Math.min(upDownSquared, 1);
    Math.max(rightLeftSquared, -1);
    Math.min(rightLeftSquared, 1);

    myDriveTrain.driveR(R*-1);
    myDriveTrain.driveL(L*-1);
    }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}