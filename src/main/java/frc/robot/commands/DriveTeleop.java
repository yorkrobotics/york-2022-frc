// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveControlState;

public class DriveTeleop extends CommandBase {
  private DriveTrain mDrive;
  private double[] arcadeDriveSpeed;

  /** Creates a new DriveWithVelocityControl. */
  public DriveTeleop(DriveTrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrive = d;
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch(mDrive.getDriveControlState()){
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
    arcadeDriveSpeed = mDrive.mArcadeDrive(RobotContainer.mController);

    if (mDrive.getDriveControlState() == DriveControlState.OPEN_LOOP){
      mDrive.setOpenLoop(arcadeDriveSpeed[0], arcadeDriveSpeed[1]);
      }
      
    if (mDrive.getDriveControlState() == DriveControlState.VELOCITY_CONTROL){
      mDrive.setVelocity(arcadeDriveSpeed[0], arcadeDriveSpeed[1]);
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
