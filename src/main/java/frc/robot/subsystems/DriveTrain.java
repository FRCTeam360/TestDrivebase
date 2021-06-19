// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrainConstants.*;
//import frc.robot.Constants.AutoConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;


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

  //private final DifferentialDriveOdometry m_odometry;
  private final SpeedControllerGroup leftGroup;
  private final SpeedControllerGroup rightGroup;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    motorLMaster = new MyTalonFX(motorLMasterID);
    motorLSlave1 = new MyTalonFX(motorLSlave1ID);
    motorLSlave2 = new MyTalonFX(motorLSlave2ID);
    motorRMaster = new MyTalonFX(motorRMasterID);
    motorRSlave1 = new MyTalonFX(motorRSlave1ID);
    motorRSlave2 = new MyTalonFX(motorRSlave2ID);

    motorLMaster.configFactoryDefault();
    motorLSlave1.configFactoryDefault();
    motorLSlave2.configFactoryDefault();
    motorRMaster.configFactoryDefault();
    motorRSlave1.configFactoryDefault();
    motorRSlave2.configFactoryDefault(); 

    // makes the second motor for left and right sides to follow the primary motor on the left and right
    motorLSlave1.follow(motorLMaster);
    motorLSlave2.follow(motorLMaster);
    motorRSlave1.follow(motorRMaster);
    motorRSlave2.follow(motorRMaster);

    // makes one side of the robot reverse direction in order to ensure that the robot goes forward when the joysticks are both forward and backwards when the joysticks are both backwards
    motorLMaster.setInverted(TalonFXInvertType.CounterClockwise);
    motorLSlave1.setInverted(TalonFXInvertType.FollowMaster);
    motorLSlave2.setInverted(TalonFXInvertType.FollowMaster);
    motorRMaster.setInverted(TalonFXInvertType.Clockwise);
    motorRSlave1.setInverted(TalonFXInvertType.FollowMaster);
    motorRSlave2.setInverted(TalonFXInvertType.FollowMaster);

    //m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    resetEncPos(); //Reset Encoders r navX yaw before m_odometry is defined

    leftGroup = new SpeedControllerGroup( motorLMaster , motorLSlave1, motorLSlave2 );
    rightGroup = new SpeedControllerGroup( motorRMaster , motorRSlave1, motorRSlave2 );

    m_differentialDrive = new DifferentialDrive(leftGroup, rightGroup);
    m_differentialDrive.setSafetyEnabled(false); //So it won't stop the motors from moving
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftGroup.setVoltage(leftVolts); //Answer is no   //Set to motor groups
    rightGroup.setVoltage(rightVolts); //it's big brain time
    m_differentialDrive.feed(); //Feed the motorsafety class so it doesnt disable the motors
  }

  public void resetEncPos () { //For initialization resets encoder positions, for ramsete
    motorLMaster.setSelectedSensorPosition(0);
    motorRMaster.setSelectedSensorPosition(0);
    // navX.zeroYaw();
    // navX.setAngleAdjustment( -navX.getAngle() ); //Set angle offset
    // m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(getHeading())); //Set odomentry to zero
  }

  // public double getHeading() {
  //   return Math.IEEEremainder(navX.getAngle(), 360) * (AutoConstants.kGyroReversed ? -1.0 : 1.0);
  // }

  public void driveR (double Rmotor) {
    motorRMaster.set( Rmotor );
  }
  public void driveL (double Lmotor) {
    motorLMaster.set( Lmotor );
  }

  // public Pose2d getPose() {
  //   return m_odometry.getPoseMeters();
  // }

  public void brakeMode() {
    motorLMaster.setNeutralMode(NeutralMode.Brake);
    motorLSlave1.setNeutralMode(NeutralMode.Brake);
    motorLSlave2.setNeutralMode(NeutralMode.Brake);
    motorRMaster.setNeutralMode(NeutralMode.Brake);
    motorRSlave1.setNeutralMode(NeutralMode.Brake);
    motorRSlave2.setNeutralMode(NeutralMode.Brake);
  }
  public void coastMode() {
    motorLMaster.setNeutralMode(NeutralMode.Coast);
    motorLSlave1.setNeutralMode(NeutralMode.Coast);
    motorLSlave2.setNeutralMode(NeutralMode.Coast);
    motorRMaster.setNeutralMode(NeutralMode.Coast);
    motorRSlave1.setNeutralMode(NeutralMode.Coast);
    motorRSlave2.setNeutralMode(NeutralMode.Coast);
  }

  // public double getHighestVelocity () { 
  //   double leftSpeed = motorLMaster.TalonFXSensorCollection.getIntegratedSensorVelocity() * AutoConstants.ticksToMeters;
  //   double rightSpeed = motorRMaster.getEncoder().getVelocity() * AutoConstants.ticksToMeters;
  //   double highSpeed = Math.max( Math.abs(leftSpeed), Math.abs(rightSpeed) ); //Make em both positive
  //   return highSpeed; //In meters per second
  // 5}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
