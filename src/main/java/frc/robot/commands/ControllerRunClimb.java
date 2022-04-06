// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.ClimbMode;

public class ControllerRunClimb extends CommandBase {
  private Climb mClimb = Climb.getInstance();
  private XboxController mController;

  /** Creates a new ControllerRunClimb. */
  public ControllerRunClimb(XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    mController = controller;
    addRequirements(mClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerValue = mController.getLeftY();
    controllerValue = MathUtil.applyDeadband(controllerValue, 0.1);

    if (mClimb.getControlMode() == ClimbMode.OPEN_LOOP){
      mClimb.runClimbOpenLoop(controllerValue * 0.2);
    }
    if (mClimb.getControlMode() == ClimbMode.POSITION_CONTROL){
      mClimb.updateSetpoint(controllerValue * 2.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimb.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
