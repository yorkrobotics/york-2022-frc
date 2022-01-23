// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFront = new CANSparkMax(1, MotorType.kBrushless);
  private CANSparkMax leftBack = new CANSparkMax(2, MotorType.kBrushless);
  private CANSparkMax rightFront = new CANSparkMax(4, MotorType.kBrushless);
  private CANSparkMax rightBack = new CANSparkMax(3, MotorType.kBrushless);

  private RelativeEncoder encoder = leftFront.getEncoder();
  private final DifferentialDrive drive;


  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);
    leftFront.setInverted(false);
    rightFront.setInverted(true);

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
    drive.arcadeDrive(-(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * speed, controller.getLeftX()*speed);

  }

  public void driveForward (double speed){
    drive.tankDrive(speed, speed);
  }

  public void stop(){
    drive.stopMotor();
  }

  public double getMetersDistance(){
    // System.out.println(encoder.getPosition() / Constants.GEAR_RATIO * 0.145 * Math.PI);
    return encoder.getPosition() / Constants.GEAR_RATIO * 0.145 * Math.PI; //0.145 is the wheel diameter
  }

  public void resetEncoder(){
    encoder.setPosition(0);
  }

}
