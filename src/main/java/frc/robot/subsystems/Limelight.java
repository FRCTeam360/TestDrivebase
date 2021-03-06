package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  NetworkTableEntry camMode = table.getEntry("camMode");
  
  /**
   * Creates a new Limelight.
   */
  public Limelight() {
  }

  public void changeCamMode() {
    if(camMode.getDouble(0.0) == 1.0) {
      table.getEntry("camMode").setNumber(0.0);
      table.getEntry("ledMode").setNumber(0);
    } else {
      table.getEntry("camMode").setNumber(1.0);
      table.getEntry("ledMode").setNumber(1);
    }
  }

  public double getCamMode() {
    return camMode.getDouble(0.0);
  }

  public boolean validTarget() {
    return tv.getDouble(0.0) == 1.0;
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  @Override
  public void periodic() {     // This method will be called once per scheduler run
    //SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
    //SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));

    SmartDashboard.putBoolean("Lime Target", validTarget()); //Comp Setup
    SmartDashboard.putNumber("Lime X", tx.getDouble(0.0)); //Comp Setup
    //Potentially add limelight display here
  }
}