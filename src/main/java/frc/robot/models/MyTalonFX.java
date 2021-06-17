// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.models;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.SpeedController;

/** Add your docs here. */
public class MyTalonFX extends TalonFX implements SpeedController {
    private int defaultPidIndex = 0;
    private double currentSetValue = 0;
    private TalonFXSensorCollection myTalonFXSensors;
    
    public MyTalonFX(int deviceNumber) {
        super(deviceNumber);

        this.myTalonFXSensors = new TalonFXSensorCollection(this);
    }

    public double getVelocity() {
        return myTalonFXSensors.getIntegratedSensorVelocity();
    }

    @Override
    public void pidWrite(double output) {
        this.currentSetValue = output;
        super.set(ControlMode.PercentOutput, this.currentSetValue);
    }

    @Override
    public void set(double speed) {
        this.currentSetValue = speed;
        super.set(ControlMode.PercentOutput, this.currentSetValue);
    }

    @Override
    public double get() {
        return this.currentSetValue;
    }

    @Override
    public void disable() {
        this.currentSetValue = 0;
        super.set(ControlMode.Disabled, this.currentSetValue);
    }

    @Override
    public void stopMotor() {
        this.currentSetValue = 0;
        super.set(ControlMode.PercentOutput, this.currentSetValue);
    }
}
