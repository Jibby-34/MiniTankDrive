// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.beans.Encoder;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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
  // TODO: Build the robot and figure out trackwidth
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.25);

  // Configure placeholder PoseEstimator
  DifferentialDrivePoseEstimator  poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d(0, 0, new Rotation2d()));

  public DrivetrainSubsystem() {

  }

  @Override
  public void periodic() {
    
  }

}
