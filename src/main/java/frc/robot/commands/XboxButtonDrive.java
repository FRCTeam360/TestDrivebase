/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class XboxButtonDrive extends CommandBase {

  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;

  public XboxButtonDrive(DriveTrain driveTrain) {

    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain); // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {   // Called when the command is initially scheduled.
  }

  @Override
  public void execute() {   // Called every time the scheduler runs while the command is scheduled.
    if(driverCont.getXButton()){

    myDriveTrain.driveR(.2);
    myDriveTrain.driveL(.2);

     
    }else{
      myDriveTrain.driveR(0);
      myDriveTrain.driveL(0);
    }
    if(driverCont.getAButton()){

      myDriveTrain.driveR(-.2);
      myDriveTrain.driveL(-.2);
       
    }else{
      myDriveTrain.driveL(0);
      myDriveTrain.driveR(0);
    }
  }   
  
  @Override
  public void end(boolean interrupted) {   // Called once the command ends or is interrupted.
  }

  @Override
  public boolean isFinished() {  // Returns true when the command should end.
    return false;
  }
}