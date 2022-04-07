// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PyCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoVisionShoot extends SequentialCommandGroup {
  /** Creates a new AutoVisionShoot. */
  private final PyCamera pycam = PyCamera.getInstantce();
  private final DriveTrain drive = DriveTrain.getInstance();
  ShootBallAndAngleTowerSequence shootBallSequence = new ShootBallAndAngleTowerSequence(0);

  public AutoVisionShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(drive::shiftToLowGear, drive),
      new RotateToTarget(drive, pycam),
      new WaitCommand(0.2),
      new InstantCommand(()->{
        shootBallSequence.setVelocity(pycam.calcVelocity());
        shootBallSequence.setAngle(pycam.calcAngle());
      }),
      shootBallSequence
    );
  }
}