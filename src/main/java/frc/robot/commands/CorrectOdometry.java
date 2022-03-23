// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PyCamera;

public class CorrectOdometry extends CommandBase {
  /** Creates a new CorrectOdometry. */
  PyCamera pycam; 
  DriveTrain mDrive;
  boolean isDone = false;

  public CorrectOdometry(PyCamera cam, DriveTrain dt) {
    pycam = cam;
    mDrive = dt;
    addRequirements(pycam, mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double x, y;
    // Rotation2d currentHeading = new Rotation2d(mDrive.getHeading() / 180 * Math.PI);
    // x = pycam.getFieldX(currentHeading);
    // y = pycam.getFieldY(currentHeading);

    // Pose2d poseMeters = new Pose2d(x, y, currentHeading);
    // Rotation2d gyroAngle = new Rotation2d(mDrive.getGyroAngle() / 180 * Math.PI);

    // mDrive.getOdometry().resetPosition(poseMeters, gyroAngle);
    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
