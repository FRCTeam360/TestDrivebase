/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.OIConstants.*;

public class XboxTankDrive extends CommandBase {
  
    private final DriveTrain myDriveTrain;
  
    private final XboxController driverCont;
  
    public XboxTankDrive(DriveTrain driveTrain) {
  
      driverCont = new XboxController(driverContPort);
  
      myDriveTrain = driveTrain;
  
      addRequirements(myDriveTrain);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(-1 *driverCont.getY(Hand.kLeft)) >= xboxDeadzone) {
      myDriveTrain.driveL(driverCont.getY(Hand.kLeft));
    }else{ myDriveTrain.driveL(0);}

    if(Math.abs(-1 *driverCont.getY(Hand.kRight)) >= xboxDeadzone) {
      myDriveTrain.driveR(driverCont.getY(Hand.kRight));
    }else{ myDriveTrain.driveR(0);}
    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
