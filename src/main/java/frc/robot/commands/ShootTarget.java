// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.Shooter;

public class ShootTarget extends CommandBase {
  private final PyCamera pycam;
  private final Shooter shooter;
  private boolean isDone;
  /** Creates a new getHoopCenter. */
  public ShootTarget(PyCamera pc, Shooter st) {
    pycam = pc;
    shooter = st;
    addRequirements(pycam, shooter);
    isDone = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(pycam.getHoopCenter()[0]);
    System.out.println(pycam.getHoopCenter()[1]);
    System.out.println(pycam.getHoopCenter()[2]);
    double angle = pycam.getAngle(shooter.getSpeed());
    System.out.println(angle);
    shooter.setAngle(angle);
    isDone = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
 }
}
