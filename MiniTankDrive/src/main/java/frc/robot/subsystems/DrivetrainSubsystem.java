// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.pathplanner.lib.auto.AutoBuilder;

public class DrivetrainSubsystem extends SubsystemBase {

  // Create motors
  PWMSparkMax leftMotor = new PWMSparkMax(Constants.LEFT_DRIVE_MOTOR_PORT);
  PWMSparkMax rightMotor = new PWMSparkMax(Constants.LEFT_DRIVE_MOTOR_PORT);

  // Create encoders
  Encoder leftEncoder = new Encoder(Constants.LEFT_DRIVE_ENCODER_PORT_A, Constants.LEFT_DRIVE_ENCODER_PORT_A);
  Encoder rightEncoder = new Encoder(Constants.RIGHT_DRIVE_ENCODER_PORT_A, Constants.RIGHT_DRIVE_ENCODER_PORT_B);


  // Combine motors into drivetrain
  DifferentialDrive drivetrain = new DifferentialDrive(leftMotor, rightMotor);
  // TODO: Build the robot and figure out trackwidth
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.25);

  // Configure placeholder pose estimator
  DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d(0, 0, new Rotation2d()));

  DifferentialDriveWheelPositions wheelPositions;

  

  public DrivetrainSubsystem() {
    // 0.66 degrees per encoder pulse (540 pulses per 360 degrees)
    leftEncoder.setDistancePerPulse(0.66);
    rightEncoder.setDistancePerPulse(0.66);

    AutoBuilder.configureRamsette();
  }

  @Override
  public void periodic() {
    // Update wheel positions and pose estimator
    wheelPositions = new DifferentialDriveWheelPositions(leftEncoder.get(), rightEncoder.get());
    poseEstimator.update(new Rotation2d(), wheelPositions);
  }

  public void drive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  public double getLeftEncoder() {
    return leftEncoder.getDistance();
  }

  public double getRightEncoder() {
    return leftEncoder.getDistance();
  }

}
