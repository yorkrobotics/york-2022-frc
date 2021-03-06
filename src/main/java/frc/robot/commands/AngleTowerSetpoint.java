// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class AngleTowerSetpoint extends CommandBase {
  private Tower mTower = Tower.getInstance();
  private double mSetAngle;
  /** Creates a new AngleTowerSetpoint. */
  public AngleTowerSetpoint(double setAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSetAngle = setAngle;
    addRequirements(mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mTower.setTowerAngle(mSetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTower.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTower.isAtAngle(mSetAngle);
  }

public void setSetpoint(double angle) {
  mSetAngle = angle;
}
}
