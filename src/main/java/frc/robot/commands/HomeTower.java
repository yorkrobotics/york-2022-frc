// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class HomeTower extends CommandBase {
  private Tower mTower = Tower.getInstance();
  /** Creates a new TowerGoHome. */
  public HomeTower() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mTower.isHome()){
      mTower.runActuatorsOpenLoop(-0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTower.stopMotors();
    mTower.zeroSetpoint();
    mTower.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTower.isHome();
  }
}
