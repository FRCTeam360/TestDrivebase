// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.OIConstants.*;

public class ButtonDriveJulia extends CommandBase {
  /** Creates a new ButtonDriveJulia. */

  private final DriveTrain myDriveTrain;

  private final XboxController driverCont;
  public ButtonDriveJulia(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    driverCont = new XboxController(driverContPort);

    myDriveTrain = driveTrain;

    addRequirements(myDriveTrain);  // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driverCont.getYButton()){
      myDriveTrain.driveR(0.2);
      myDriveTrain.driveL(0.2);
    }else if(driverCont.getXButton()){
      myDriveTrain.driveL(-0.2);
      myDriveTrain.driveR(-0.2);
    }else{
      myDriveTrain.driveL(0);
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
