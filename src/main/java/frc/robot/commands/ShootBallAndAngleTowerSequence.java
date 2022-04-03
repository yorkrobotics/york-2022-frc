// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBallAndAngleTowerSequence extends SequentialCommandGroup {
  /** Creates a new RunShooterSequence. */
  public ShootBallAndAngleTowerSequence(double velocity, double angle) {
    addCommands(
      new ReverseConveyorTimed(0.25),
      new ParallelCommandGroup(
        new AngleTowerSetpoint(angle),
        new RunShooterPIDToSetpoint(velocity)
      ),
      new ParallelRaceGroup(
        new RunShooterPID(velocity),
        new RunConveyorTimed(1)
      )
    );
  }
}
