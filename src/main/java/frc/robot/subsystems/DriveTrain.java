// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  PWMSparkMax leftFront;
  PWMSparkMax rightFront;
  PWMSparkMax leftBack;
  PWMSparkMax rightBack;
  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;
  DifferentialDrive drive;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFront = new PWMSparkMax(Constants.LEFT_FRONT);
    leftFront.setInverted(false);
    rightFront = new PWMSparkMax(Constants.RIGHT_FRONT);
    rightFront.setInverted(true);

    // leftBack = new PWMSparkMax(Constants.LEFT_BACK)
    // leftBack.setInverted(false);
    // rightBack = new PWMSparkMax(Constants.RIGHT_BACK);
    // rightBack.setInverted(false);
    // leftMotors = new MotorControllerGroup(leftFront, leftBack);
    // rightMotors = new MotorControllerGroup(rightFront, rightBack);
        // NOTE: test the motors one by one

    drive = new DifferentialDrive(leftFront, rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoysticks (XboxController controller, double speed){
    // drive.arcadeDrive(controller.getRawAxis(Constants.X_BOX_Y_AXIS)*speed, controller.getRawAxis(Constants.X_BOX_X_AXIS)*speed);
    drive.arcadeDrive(controller.getLeftY()*speed, controller.getLeftX()*speed);

  }

  public void driveWithTriggers (XboxController controller, double speed){
    // drive.arcadeDrive(controller.getRawAxis(Constants.X_BOX_Y_AXIS)*speed, controller.getRawAxis(Constants.X_BOX_X_AXIS)*speed);
    drive.arcadeDrive((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * speed, controller.getLeftX()*speed);

  }

  public void Turning (XboxController controller, double speed){
    // drive.arcadeDrive(0, controller.getRawAxis(Constants.X_BOX_X_AXIS)*speed);
    drive.arcadeDrive(0, controller.getLeftX()*speed);
  }

  public void driveTimed(double speed){
    drive.tankDrive(speed, speed);
  }

  public void stop(){
    drive.stopMotor();
  }
}
