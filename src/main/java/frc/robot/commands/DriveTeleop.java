// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.DriveControlMode;

public class DriveTeleop extends CommandBase {
  private DriveTrain mDrive = DriveTrain.getInstance();
  private WheelSpeeds mWheelSpeeds;
  private XboxController mController;

  private SlewRateLimiter mLimiterHighGear = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT_HIGH_GEAR);
  private SlewRateLimiter mLimiterLowGear = new SlewRateLimiter(Constants.SLEW_RATE_LIMIT_LOW_GEAR);

  /** Creates a new DriveWithVelocityControl. */
  public DriveTeleop(XboxController controller) {
    mController = controller;
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwards = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();
    double limitedForwards = forwards;

    switch (mDrive.getGearMode()) {
      case HIGH_GEAR:
        limitedForwards = mLimiterHighGear.calculate(forwards);
        break;
      case LOW_GEAR:
        limitedForwards = mLimiterLowGear.calculate(forwards);
        break;
      case UNKNOWN:
        break;
      default:
        break;
    }

    if (mDrive.isInvertedDriving()){
      limitedForwards = - limitedForwards;
    }

    mWheelSpeeds = DifferentialDrive.arcadeDriveIK(limitedForwards, mController.getLeftX(), true);

    if (mDrive.getDriveControlMode() == DriveControlMode.OPEN_LOOP){
      mDrive.driveOpenloop(mWheelSpeeds.left, mWheelSpeeds.right);
      }
    if (mDrive.getDriveControlMode() == DriveControlMode.VELOCITY_CONTROL){
      mDrive.driveVelocitySetpoint(mWheelSpeeds.left, mWheelSpeeds.right);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.stopDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
