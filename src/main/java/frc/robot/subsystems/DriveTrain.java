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
  private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder encoder = leftMotor.getEncoder();
  private final DifferentialDrive drive;


  /** Creates a new DriveTrain. */
  public DriveTrain() {

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);

    drive = new DifferentialDrive(leftMotor, rightMotor);
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

  public void driveForward (double speed){
    drive.tankDrive(speed, speed);
  }

  public void stop(){
    drive.stopMotor();
  }

  public double getMetersDistance(){
    return encoder.getPosition() * Constants.GEAR_RATIO * 15.24 * Math.PI;
  }
}
