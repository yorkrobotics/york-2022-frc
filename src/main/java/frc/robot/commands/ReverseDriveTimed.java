// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ReverseDriveTimed extends CommandBase {
  /** Creates a new AutoDrive. */
  private DriveTrain mDriveTrain = DriveTrain.getInstance();
  private Timer mTimer = new Timer();
  private double seconds;
  public ReverseDriveTimed(double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDriveTrain);
    this.seconds = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.reset();
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveTrain.driveOpenloop(-0.5, -0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDriveTrain.stopDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(seconds);
  }
}
