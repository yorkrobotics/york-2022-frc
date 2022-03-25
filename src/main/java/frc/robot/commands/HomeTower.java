// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class HomeTower extends CommandBase {
  /** Creates a new HomeTower. */
  Tower mTower;
  public HomeTower(Tower tw) {
    mTower = tw;
    addRequirements(tw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTower.runActuatorsPositionControl(0);
    mTower.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (!mLeftLimitSwitch.isPressed()) {
    //   mLeftMotor.set(-0.2);
    // }
    // if (!mRightLimitSwitch.isPressed()) {
    //   mRightMotor.set(-0.2);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (mLeftLimitSwitch.isPressed() && mRightLimitSwitch.isPressed());
    return false;
  }
}
