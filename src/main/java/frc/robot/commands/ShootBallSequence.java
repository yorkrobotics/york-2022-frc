// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBallSequence extends SequentialCommandGroup {

  RunShooterPIDToSetpoint runShooter = new RunShooterPIDToSetpoint(0);
  RunShooterPID runShooterNoSetpoint = new RunShooterPID(0);
  AngleTowerVision angleTowerVision = new AngleTowerVision();

  /** Creates a new ShootBallSequence. */
  public ShootBallSequence(double velocity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      angleTowerVision,
      new ReverseConveyorTimed(0.25),
      runShooter,
      new ParallelRaceGroup(
        runShooterNoSetpoint,
        new RunConveyorTimed(1.5)
      )
    );
  }

  public void setVelocity(double velocity) {
    runShooter.setVelocity(velocity);
    runShooterNoSetpoint.setVelocity(velocity);
  }
}
