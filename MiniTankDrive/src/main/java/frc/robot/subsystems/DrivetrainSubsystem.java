// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {

  // Create motors and drivetrain
  PWMSparkMax leftMotor = new PWMSparkMax(Constants.LEFT_DRIVE_MOTOR_PORT);
  PWMSparkMax rightMotor = new PWMSparkMax(Constants.LEFT_DRIVE_MOTOR_PORT);
  DifferentialDrive drivetrain = new DifferentialDrive(leftMotor, rightMotor);

  public DrivetrainSubsystem() {

  }

  @Override
  public void periodic() {
    
  }

}
