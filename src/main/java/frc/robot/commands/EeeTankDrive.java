/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import static frc.robot.Constants.OIConstants.*;

public class EeeTankDrive extends CommandBase {

    private final DriveTrain myDriveTrain;
    private final XboxController driveCont;

  /**
   * Creates a new EeeTankDrive.
   */
  public EeeTankDrive(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
        driveCont = new XboxController(driverContPort);

        myDriveTrain = driveTrain;

        addRequirements(myDriveTrain); //use addRequirements() here to declare subsystems dependencies
    }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double lValue = this.driveCont.getY(Hand.kLeft);
    double rValue = this.driveCont.getY(Hand.kRight);

    myDriveTrain.driveL(lValue);
    myDriveTrain.driveR(rValue);

    
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
