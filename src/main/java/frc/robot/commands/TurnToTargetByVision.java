// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PyCamera;


public class TurnToTargetByVision extends CommandBase {
  private PyCamera pycam;
  private DriveTrain mDrive;
  private boolean isFinished = false, isEnabled = false;
  private Timer timer = new Timer();
  /** Creates a new TurnToTargetByVision. */
  public TurnToTargetByVision(PyCamera pc, DriveTrain d) {
    pycam = pc;
    mDrive = d;
    addRequirements(pycam, mDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    // mDrive.configurePositionControl();

    // NetworkTableEntry hoopCenterEntry = NetworkTableInstance.getDefault().getTable("Vision").getEntry("translation_vector");
    // hoopCenterEntry.addListener(event -> {
    //   if (isEnabled) mDrive.setRotation(pycam.getHorizontalAngle());
    //   double angle = pycam.getHorizontalAngle();
    //   SmartDashboard.putNumber("target rel angle", angle);
    // }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = pycam.getHorizontalAngle();
    double currentHeading = mDrive.getHeading();
    double headingAngle = currentHeading + angle;
    if (headingAngle < 0) {
      headingAngle += 360;
    }

    if (timer.hasElapsed(1)) {
      mDrive.turnToHeadingAngle(headingAngle);
      timer.reset();
    }
    SmartDashboard.putNumber("target rel angle", angle);
    SmartDashboard.putNumber("target heading angle", headingAngle);
    // isEnabled = Math.abs(pycam.getHorizontalAngle()) > 2;
    // isFinished = !isEnabled;
    // isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
