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

  private double x_field; 
  private double power;
  private double targetAngle;
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
    x_field = pycam.getFieldX();
    targetAngle = 1 / Math.pow((0.000896333 * x_field + 0.00200909), 1.14638) + 41.2633;
    power = 0.513798 + 0.000825723 * x_field - 0.00000323753 * Math.pow(x_field, 2) + 8.5563 * Math.pow(10, -9) * Math.pow(x_field, 3);
    SmartDashboard.putNumber("target angle", targetAngle);
    SmartDashboard.putNumber("velocity", power);
    tower.setTowerAngle(targetAngle);
    shooter.runShooter(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double towerAngle = tower.getTowerAngle();
    // double y_field = pycam.getFieldY();

    // double minAngle = Math.atan(y_field/x_field);
    // if (towerAngle < minAngle) { // 55 - 10 is the smallest angle
    //   if (minAngle > 55) {
    //     tower.setTowerAngle(minAngle + 10); // add 10 to create a better projectile
    //     towerAngle = minAngle + 10;
    //   } else {
    //     this.cancel();
    //   }
    // }
    // double velocity = pycam.calcVelocity();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.stopConveyor();
    shooter.stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
 }
}
