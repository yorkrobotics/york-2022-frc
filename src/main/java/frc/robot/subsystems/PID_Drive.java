// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class PID_Drive extends PIDSubsystem {
  /** Creates a new PID_Drive. */
  private CANSparkMax leftMotor = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(1, MotorType.kBrushless);
  private RelativeEncoder encoder = leftMotor.getEncoder();
  DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  public PID_Drive() {
    super(
        // The PIDController used by the subsystem
        new PIDController(Constants.DRIVE_KP, Constants.DRIVE_KI, Constants.DRIVE_KD));
    setSetpoint(Constants.TARGET_DISTANCE);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    leftMotor.set(output);
    rightMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getPosition() / Constants.GEAR_RATIO * 15.24 * Math.PI;
  }

  public void stop() {
    drive.stopMotor();
  }
}
