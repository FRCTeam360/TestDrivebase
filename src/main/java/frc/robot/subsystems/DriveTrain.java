// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.*;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.models.MyTalonFX;

public class DriveTrain extends SubsystemBase {
  
  private static MyTalonFX motorLMaster;
  private static MyTalonFX motorLSlave1;
  private static MyTalonFX motorLSlave2;
  private static MyTalonFX motorRMaster;
  private static MyTalonFX motorRSlave1;
  private static MyTalonFX motorRSlave2;

  private final DifferentialDrive m_differentialDrive;

  private double leftVel;   // initializes velocities for left and right sides
  private double rightVel;
  private double leftNewPos;   // initializes new positions for left and right sides
  private double rightNewPos;

  private final DifferentialDriveOdometry m_odometry;
  private final SpeedControllerGroup leftGroup;
  private final SpeedControllerGroup rightGroup;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    motorLMaster = new MyTalonFX(motorLMasterID);
    motorLSlave1 = new MyTalonFX(motorLMasterID);
    motorLSlave2 = new MyTalonFX(motorLMasterID);
    motorRMaster = new MyTalonFX(motorLMasterID);
    motorRSlave1 = new MyTalonFX(motorLMasterID);
    motorRSlave2 = new MyTalonFX(motorLMasterID);

    leftGroup = new SpeedControllerGroup( motorLMaster , motorLSlave1, motorLSlave2 );
    rightGroup = new SpeedControllerGroup( motorRMaster , motorRSlave1, motorRSlave2 );

    m_differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
