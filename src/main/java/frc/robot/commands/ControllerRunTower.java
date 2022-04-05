// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Tower.TowerActuatorMode;

public class ControllerRunTower extends CommandBase {
  private Tower mTower = Tower.getInstance();
  private XboxController mController;
  /** Creates a new RunTower. */
  public ControllerRunTower(XboxController controller) {
    mController = controller;
    addRequirements(mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerValue = mController.getRightY();
    controllerValue = MathUtil.applyDeadband(controllerValue, 0.1);

    if (mTower.getControlMode() == TowerActuatorMode.OPEN_LOOP){
      mTower.runActuatorsOpenLoop(controllerValue * 0.2);
    }

    if (mTower.getControlMode() == TowerActuatorMode.POSITION_CONTROL){
      mTower.updateSetpoint(controllerValue * 1.8);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTower.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
