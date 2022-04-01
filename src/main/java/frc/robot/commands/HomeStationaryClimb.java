// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StationaryClimb;

public class HomeStationaryClimb extends CommandBase {
  private StationaryClimb mStationaryClimb = StationaryClimb.getInstance();
  /** Creates a new HomeStationaryClimb. */
  public HomeStationaryClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mStationaryClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mStationaryClimb.isHome()){
      mStationaryClimb.runClimbOpenLoop(-0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mStationaryClimb.stopMotors();
    mStationaryClimb.zeroSetpoint();
    mStationaryClimb.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mStationaryClimb.isHome();
  }
}
