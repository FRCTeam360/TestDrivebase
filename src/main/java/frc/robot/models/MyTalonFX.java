// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.models;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/** Add your docs here. */
public class MyTalonFX extends TalonFX {
    public MyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }
}
