// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StationaryClimb;
import frc.robot.subsystems.StationaryClimb.StationaryClimbMode;

public class ControllerRunStationaryClimb extends CommandBase {
  private StationaryClimb mStationaryClimb = StationaryClimb.getInstance();
  private XboxController mController;

  /** Creates a new RunClimbController. */
  public ControllerRunStationaryClimb(XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    mController = controller;
    addRequirements(mStationaryClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerValue = - mController.getRightY();
    controllerValue = MathUtil.applyDeadband(controllerValue, 0.1);

    if (mStationaryClimb.getControlMode() == StationaryClimbMode.OPEN_LOOP){
      mStationaryClimb.runClimbOpenLoop(controllerValue * 0.2);
    }
    if (mStationaryClimb.getControlMode() == StationaryClimbMode.POSITION_CONTROL){
      mStationaryClimb.updateSetpoint(controllerValue * 1.5);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mStationaryClimb.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
