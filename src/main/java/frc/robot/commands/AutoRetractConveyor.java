// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;

public class AutoRetractConveyor extends CommandBase {

  private Conveyor mConveyor;
  private Timer mTimer;
  /** Creates a new AutoRetractConveyor. */
  public AutoRetractConveyor(Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    mConveyor = conveyor;
    mTimer = new Timer();
    mTimer.reset();
    addRequirements(mConveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mConveyor.runConveyor(-Constants.CONVEYOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mConveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.hasElapsed(0.25);
  }
}
