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

  RunShooterPIDToSetpoint runShooter = new RunShooterPIDToSetpoint(0);
  RunShooterPID runShooterNoSetpoint = new RunShooterPID(0);
  AngleTowerSetpoint angleTowerSetpoint = new AngleTowerSetpoint(0);

  /** Creates a new ShootBallSequence. */
  public ShootBallAndAngleTowerSequence(double velocity) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        angleTowerSetpoint,
        new SequentialCommandGroup(
          new ReverseConveyorTimed(0.25),
          runShooter
        )
      ),
      new ParallelRaceGroup(
        runShooterNoSetpoint,
        new RunConveyorTimed(1.5)
      )
    );
  }

  public void setVelocity(double velocity) {
    System.out.println("set velocity to " + velocity);
    runShooter.setVelocity(velocity);
    runShooterNoSetpoint.setVelocity(velocity);
  }
  public void setAngle(double angle) {
    System.out.println("set angle to " + angle);
    angleTowerSetpoint.setSetpoint(angle);
  }
  // TODO set angle
}
