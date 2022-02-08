// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.FeedIntake;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class feed extends CommandBase {
  private FeedIntake feedSystem;

  /**
   *
   * @param sub The subsystem used by this command.
   */
  public feed(FeedIntake sub) {
    feedSystem = sub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feedSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feedSystem.setSpeed(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feedSystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}