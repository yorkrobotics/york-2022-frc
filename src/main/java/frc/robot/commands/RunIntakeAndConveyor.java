// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeLifterMode;

public class RunIntakeAndConveyor extends CommandBase {
  /** Creates a new RunIntakeAndConveyor. */
  private final Intake mIntake;
  private final Shooter mShooter;

  public RunIntakeAndConveyor(Intake intake, Shooter shooter) {
    mIntake = intake;
    mShooter = shooter;
    addRequirements(mIntake);
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mIntake.getIntakeLifterMode() != IntakeLifterMode.DEPLOYED) return;

    mIntake.runRoller(Constants.INTAKE_ROLLER_SPEED);
    mShooter.runConveyor(Constants.CONVEYOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIntake.stopRoller();
    mShooter.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
