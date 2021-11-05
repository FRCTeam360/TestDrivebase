// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class XboxTankDrive extends CommandBase {
  /** Creates a new XboxTankDrive. */
  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  public XboxTankDrive(DriveTrain driveTrain) {

    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {   // Called when the command is initially scheduled.
  }
 
  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
  
   if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone) {
    myDriveTrain.driveL(-1*driverCont.getY(Hand.kLeft));
   }
   if(Math.abs(driverCont.getY(Hand.kRight)) >= xboxDeadzone) {
     myDriveTrain.driveR(-1*driverCont.getY(Hand.kRight));
   }
   if(Math.abs(driverCont.getY(Hand.kLeft)) < xboxDeadzone) {
     myDriveTrain.driveL(0);
   }
   if(Math.abs(driverCont.getY(Hand.kRight)) < xboxDeadzone) {
     myDriveTrain.driveR(0);
   }
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
