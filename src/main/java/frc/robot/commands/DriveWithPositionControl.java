// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveWithPositionControl extends CommandBase {
  private DriveTrain vController;

  /** Creates a new DriveWithPositionControl. */
  public DriveWithPositionControl(DriveTrain v) {
    // Use addRequirements() here to declare subsystem dependencies.
    vController = v;
    addRequirements(vController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    vController.driveWithPositionControl(RobotContainer.m_controller, Constants.TARGET_DISTANCE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}