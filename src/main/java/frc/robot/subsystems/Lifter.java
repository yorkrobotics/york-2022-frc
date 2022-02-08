// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lifter extends SubsystemBase {
  private CANSparkMax mLiftMotor;
  private RelativeEncoder mEncoder;

  private SparkMaxPIDController mLiftController;

  private double kP, kI, kD, kIZone, kFF, kMinOutput, kMaxOutput;
  /** Creates a new Lifter. */
  public Lifter() {
    mLiftMotor = new CANSparkMax(5, MotorType.kBrushless);
    mEncoder = mLiftMotor.getEncoder();
    mLiftController = mLiftMotor.getPIDController();

    kP = 0;
    kI = 0;
    kD = 0;
    kIZone = 0;
    kFF = 0;
    kMinOutput = -0.5;
    kMaxOutput = -0.5;

    mLiftController.setP(kP);
    mLiftController.setI(kI);
    mLiftController.setD(kD);
    mLiftController.setIZone(kIZone);
    mLiftController.setFF(kFF);
    mLiftController.setOutputRange(kMinOutput, kMaxOutput);

    mLiftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mLiftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    mLiftMotor.setSoftLimit(SoftLimitDirection.kForward, 15);
    mLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lift_P", kP);
    SmartDashboard.putNumber("Lift_I", kI);
    SmartDashboard.putNumber("Lift_D", kD);
    SmartDashboard.putNumber("Lift_position", mEncoder.getPosition());
    SmartDashboard.putNumber("Lift forward limit", mLiftMotor.getSoftLimit(SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Lift reverse limit", mLiftMotor.getSoftLimit(SoftLimitDirection.kReverse));
    
    double p = SmartDashboard.getNumber("Lift_P", 0);
    double i = SmartDashboard.getNumber("Lift_I", 0);
    double d = SmartDashboard.getNumber("Lift_D", 0);

    if(p != kP) {mLiftController.setP(p); kP = p;}
    if(i != kI) {mLiftController.setI(i); kI = i;}
    if(d != kD) {mLiftController.setD(d); kD = d;}

    mLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)SmartDashboard.getNumber("Lift forward limit", 15));
    mLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)SmartDashboard.getNumber("Lift reverse limit", 0));
  }

  public void setLiftPosition(double setPoint){
    mLiftController.setReference(setPoint, ControlType.kPosition);
  }

  public void runLiftWithJoystick(XboxController controller){
    setLiftPosition(controller.getRightY());
  }

  public void resetEncoder(){
    mEncoder.setPosition(0);
  }

  public double rotationsToMeters(double rotations){
    return 0;
  }
}
