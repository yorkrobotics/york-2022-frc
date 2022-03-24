// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveControlMode;

public class DriveTeleop extends CommandBase {
  private DriveTrain mDrive;
  private WheelSpeeds mWheelSpeeds;

  /** Creates a new DriveWithVelocityControl. */
  public DriveTeleop(DriveTrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrive = d;
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mWheelSpeeds = mDrive.mArcadeDrive(RobotContainer.mainController);

    if (mDrive.getDriveControlMode() == DriveControlMode.OPEN_LOOP){
      mDrive.driveOpenloop(mWheelSpeeds.left, mWheelSpeeds.right);
      }
      
    if (mDrive.getDriveControlMode() == DriveControlMode.VELOCITY_CONTROL){
      mDrive.driveVelocitySetpoint(mWheelSpeeds.left, mWheelSpeeds.right);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.stopDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
