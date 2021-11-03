// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

import com.kauailabs.navx.frc.AHRS;


public class XboxFieldOrientedDrive extends CommandBase {
  
  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  /** Creates a new XboxFieldOrientedDrive. */
  public XboxFieldOrientedDrive(DriveTrain driveTrain) {
    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double gyroAngle = myDriveTrain.getYaw();

    double rightLeftSquared = 0;
    double upDownSquared = 0;
    double driveRight = 0;
    double driveLeft = 0;

    //sets doubled right/left value of xboxController
    if(Math.abs(driverCont.getX(Hand.kLeft)) >= xboxDeadzone) {
      rightLeftSquared = driverCont.getX(Hand.kLeft) * driverCont.getX(Hand.kLeft);
      if(driverCont.getX(Hand.kLeft) < 0){
        rightLeftSquared = rightLeftSquared * -1;
      }
    }
    //sets doubled up/down value of xboxController
    if(Math.abs(driverCont.getY(Hand.kLeft)) >= xboxDeadzone) {
      upDownSquared = -1 * driverCont.getY(Hand.kLeft) * driverCont.getY(Hand.kLeft);
      if(driverCont.getY(Hand.kLeft) < 0){
        upDownSquared = upDownSquared * -1;
      }
    }

    

    //field oriented drive conversion. forward = robot-based forward value, right = robot-based turning adjustment
    double forward = upDownSquared * Math.cos(gyroAngle) + rightLeftSquared * Math.sin(gyroAngle);
    double right = -1 * upDownSquared * Math.sin(gyroAngle) + rightLeftSquared * Math.cos(gyroAngle);

    System.out.println("forward: " + forward);
    System.out.println("right: " + right);

    //sets drive values using previous values for right/left and forward/back
    driveLeft = forward + right;
    driveRight = forward - right;

    //ensures motors are not passed value greater than 1 or less than -1
    driveLeft = Math.min(driveLeft, 1);
    driveRight = Math.min(driveRight, 1);
    driveLeft = Math.max(driveLeft, -1);
    driveRight = Math.max(driveRight, -1);
 
    myDriveTrain.driveL(driveLeft * 1.0);
    myDriveTrain.driveR(driveRight * 1.0);
    
    // double contRadians = Math.atan2(driverCont.getY(Hand.kLeft), driverCont.getX(Hand.kRight)); //arctan of stick inputs for radians
    // double lStickAngle = Math.toDegrees(contRadians); //radians to degrees 
    
    if(driverCont.getYButton()){
      myDriveTrain.resetEncPos(); //reset angle when Y pressed
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
