// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class HomeClimb extends CommandBase {
  private Climb mClimb = Climb.getInstance();
  
  /** Creates a new ClimbGoHome. */
  public HomeClimb() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mClimb.isHome()){
      mClimb.runClimbOpenLoop(0.6);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimb.stopMotors();
    mClimb.zeroSetpoint();
    mClimb.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mClimb.isHome();
  }
}
