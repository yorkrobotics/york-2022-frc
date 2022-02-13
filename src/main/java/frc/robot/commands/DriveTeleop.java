// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    switch(mDrive.getDriveControlMode()){
      case OPEN_LOOP:
        break;
      case VELOCITY_CONTROL:
        mDrive.configureVelocityControl();
        break;
      case POSITION_CONTROL:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mWheelSpeeds = mDrive.mArcadeDrive(RobotContainer.mController);

    SmartDashboard.putNumber("left_setpoint", mWheelSpeeds.left * Constants.DRIVE_MAX_RPM);
    SmartDashboard.putNumber("right_setpoint", mWheelSpeeds.right * Constants.DRIVE_MAX_RPM);


    if (mDrive.getDriveControlMode() == DriveControlMode.OPEN_LOOP){
      mDrive.setOpenLoop(mWheelSpeeds.left, mWheelSpeeds.right);
      }
      
    if (mDrive.getDriveControlMode() == DriveControlMode.VELOCITY_CONTROL){
      mDrive.setVelocity(mWheelSpeeds.left, mWheelSpeeds.right);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
