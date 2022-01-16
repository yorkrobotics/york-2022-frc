// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardDistance extends PIDCommand {
  /** Creates a new DriveForwardDistance. */
  DriveTrain driveTrain;

  public DriveForwardDistance(DriveTrain dt) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KD),
        // This should return the measurement
        () -> dt.getMetersDistance(),
        // This should return the setpoint (can also be a constant)
        () -> Constants.TARGET_DISTANCE,
        // This uses the output
        output -> {
          dt.driveForward(Math.min(1, Math.max(output, -1)));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    driveTrain = dt;
    addRequirements(driveTrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
