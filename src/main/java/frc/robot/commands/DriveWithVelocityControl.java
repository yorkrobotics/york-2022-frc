// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VelocityController;

public class DriveWithVelocityControl extends CommandBase {
  private VelocityController vController;

  /** Creates a new DriveWithVelocityControl. */
  public DriveWithVelocityControl(VelocityController v) {
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
    vController.driveWithVelocityControl(RobotContainer.m_controller);
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
