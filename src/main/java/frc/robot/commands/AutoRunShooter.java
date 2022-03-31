// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRunShooter extends SequentialCommandGroup {
  private Conveyor mConveyor;
  private Shooter mShooter;
  private Tower mTower;

  /** Creates a new AutoRunShooter. */
  public AutoRunShooter(Conveyor conveyor, Shooter shooter, Tower tower, double speed, double angle) {
    mConveyor = conveyor;
    mShooter = shooter;
    mTower = tower;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ReverseConveyorTimed(0.25), 
      new InstantCommand(()->mShooter.runShooter(speed), mShooter), 
      new InstantCommand(()->mTower.setTowerAngle(angle)),
      new InstantCommand(() -> Timer.delay(2)),
      new InstantCommand(()->mConveyor.runConveyor(Constants.CONVEYOR_SPEED), mConveyor),
      new InstantCommand(() -> Timer.delay(2))

    );
  }
}
