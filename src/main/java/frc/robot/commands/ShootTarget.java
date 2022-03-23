// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class ShootTarget extends CommandBase {
  private final PyCamera pycam;
  private final Shooter shooter;
  private final Tower tower;
  /** Creates a new getHoopCenter. */
  public ShootTarget(PyCamera pc, Shooter st, Tower tr) {
    pycam = pc;
    shooter = st;
    tower = tr;
    addRequirements(pycam, shooter, tower);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double towerAngle = tower.getTowerAngle();
    double x_field = pycam.getFieldX();
    double y_field = pycam.getFieldY();

    double minAngle = Math.atan(y_field/x_field);
    if (towerAngle < minAngle) { // 55 - 10 is the smallest angle
      if (minAngle > 55) {
        tower.setTowerAngle(minAngle + 10); // add 10 to create a better projectile
        towerAngle = minAngle + 10;
      } else {
        this.cancel();
      }
    }

    double velocity = pycam.calcVelocity();
    SmartDashboard.putNumber("shooter velocity: ", velocity);

    velocity = Double.min(.8, velocity);
    shooter.runConveyor(-0.3);
    Timer.delay(0.5);
    shooter.stopConveyor();
    shooter.runShooter(velocity * 0.8);
    Timer.delay(2);
    shooter.runConveyor(Constants.CONVEYOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopConveyor();
    shooter.stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
 }
}
