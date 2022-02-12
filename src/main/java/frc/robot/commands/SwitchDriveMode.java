// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveControlState;

public class SwitchDriveMode extends CommandBase {
  private DriveTrain mDrive;
  private boolean isSwitched;
  /** Creates a new SwitchDriveMode. */
  public SwitchDriveMode(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrive = dt;
    addRequirements(mDrive);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isSwitched = false;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mDrive.getDriveControlState() == DriveControlState.VELOCITY_CONTROL){
      mDrive.setOpenLoop();
      System.out.println("[Drive] switched to open loop");
      isSwitched = true;
    }
    else if (mDrive.getDriveControlState() == DriveControlState.OPEN_LOOP){
      mDrive.setToVelocityControl();
      mDrive.configureVelocityControl();
      System.out.println("[Drive] switched to velocity control");
      isSwitched = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isSwitched;
  }
}
