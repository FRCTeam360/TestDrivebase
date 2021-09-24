// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;


public class ButtonDrive extends CommandBase {
  /** Creates a new ButtonDrive. */
  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  public ButtonDrive(DriveTrain driveTrain) {
    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}
  if(Math.abs(driverCont.getY(Hand.kRight)) >= .125){
    myDriveTrain.driveR(-1 * driverCont.getY(Hand.kRight) * 1.0);
  }else{
    myDriveTrain.driveR(0);
  }
  if(Math.abs(driverCont.getY(Hand.kLeft)) >= .125){
    myDriveTrain.driveL(-1 * driverCont.getY(Hand.kLeft) * 1.0);
  }else{
    myDriveTrain.driveL(0);

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  public void end(boolean interrupted) {}   // Called once the command ends or is interrupted.
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}