// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CustomMotorController;

public class DrivetrainSubsystem extends SubsystemBase {
  CustomMotorController leftMotor = new CustomMotorController(9, 0, 1);

  public DrivetrainSubsystem() {

  }

  @Override
  public void periodic() {
    leftMotor.set(0.1);
  }
}
