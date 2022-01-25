// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DriveTeleop extends CommandBase {
  /** Creates a new DriveWithJoysticks. */

  private final DriveTrain driveTrain;

  public DriveTeleop(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Constants.DRIVE_MODE == "J"){
    //   driveTrain.driveWithJoysticks(RobotContainer.m_controller, Constants.DRIVETRAIN_SPEED);
    // }
    // else if (Constants.DRIVE_MODE == "T"){
    driveTrain.driveWithTriggers(RobotContainer.m_controller, Constants.DRIVETRAIN_SPEED);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
