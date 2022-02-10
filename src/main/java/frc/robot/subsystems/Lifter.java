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
  private LifterControlMode mLifterControlMode;

  private double kP, kI, kD, kIZone, kFF, kMinOutput, kMaxOutput;
  /** Creates a new Lifter. */
  public Lifter() {
    mLiftMotor = new CANSparkMax(5, MotorType.kBrushless);
    mLiftMotor.restoreFactoryDefaults();
  

    mEncoder = mLiftMotor.getEncoder();
    mLiftController = mLiftMotor.getPIDController();

    mLiftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mLiftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    mLiftMotor.setSoftLimit(SoftLimitDirection.kForward, 15);
    mLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, 5);

    mLifterControlMode = LifterControlMode.OPEN_LOOP;

    configureLifterController();

    SmartDashboard.putNumber("Lift_P", mLiftController.getP());
    SmartDashboard.putNumber("Lift_I", mLiftController.getI());
    SmartDashboard.putNumber("Lift_D", mLiftController.getD());
    SmartDashboard.putNumber("Lift forward limit", mLiftMotor.getSoftLimit(SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Lift reverse limit", mLiftMotor.getSoftLimit(SoftLimitDirection.kReverse));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lift_position", mEncoder.getPosition());
  }

  public void setPosition(double setPoint){
    mLiftController.setReference(setPoint, ControlType.kPosition);
  }

  public void runLiftWithJoystick(XboxController controller){
    updateFromSmartDashboard();
    
    if (mLifterControlMode == LifterControlMode.POSITION_CONTROL){
      setPosition(controller.getRightY() * 50);
    }
    if (mLifterControlMode == LifterControlMode.OPEN_LOOP){
      mLiftMotor.set(controller.getRightY() * 0.2);
    }
  }

  public void resetEncoder(){
    mEncoder.setPosition(0);
  }

  public double rotationsToMeters(double rotations){
    return 0;
  }

  public void switchLifterMode(){
    if (mLifterControlMode == LifterControlMode.OPEN_LOOP){
      mLifterControlMode = LifterControlMode.POSITION_CONTROL;
      System.out.println("Lifter: switched to Position Control");
    }
    else {
      mLifterControlMode = LifterControlMode.OPEN_LOOP;
      System.out.println("Lifter: switched to Open Loop");
    }
  }

  public void updateFromSmartDashboard(){
    double p = SmartDashboard.getNumber("Lift_P", 0);
    double i = SmartDashboard.getNumber("Lift_I", 0);
    double d = SmartDashboard.getNumber("Lift_D", 0);

    if(p != mLiftController.getP()) {mLiftController.setP(p);}
    if(i != mLiftController.getI()) {mLiftController.setI(i);}
    if(d != mLiftController.getD()) {mLiftController.setD(d);}

    mLiftMotor.setSoftLimit(SoftLimitDirection.kForward, (float)SmartDashboard.getNumber("Lift forward limit", 15));
    mLiftMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)SmartDashboard.getNumber("Lift reverse limit", 0));
  }

  public void configureLifterController(){
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
  }

  public enum LifterControlMode{
    OPEN_LOOP, POSITION_CONTROL
  }
}
