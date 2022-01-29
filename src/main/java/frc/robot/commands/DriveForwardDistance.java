// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardDistance extends PIDCommand {
  /** Creates a new DriveForwardDistance. */
  DriveTrainController m_drive;

  public DriveForwardDistance(DriveTrainController dt) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_kD),
        // This should return the measurement
        () -> dt.getPosition(),
        // This should return the setpoint (can also be a constant)
        () -> -(Constants.TARGET_DISTANCE),
        // This uses the output
        output -> {
          System.out.println(output);
          // dt.driveForward(Math.min(0.5, Math.max(output, -0.5)));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_drive = dt;
    addRequirements(m_drive);
  }

  @Override
  public void initialize(){
    super.initialize();
    // m_drive.resetEncoder();       ***resetEncoder() needs to be added
    getController().setTolerance(0.01);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
