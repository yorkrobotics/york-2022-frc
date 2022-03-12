// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lifter extends SubsystemBase {
  private static final Lifter mLifter = new Lifter();
  public static Lifter getInstance(){
    return mLifter;
  }

  private CANSparkMax mLeftPrimaryMotor, mRightPrimaryMotor, mLeftSecondaryMotor, mRightSecondaryMotor;
  private SparkMaxPIDController mLeftPrimaryController, mRightPrimaryController;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  // private SparkMaxLimitSwitch mLimitSwitch;
  // private DigitalInput mLimitSwitch;

  
  private LifterControlMode mLifterControlMode;

  private double kP, kI, kD, kMinOutput, kMaxOutput;

  /** Creates a new Lifter. */
  public Lifter() {
    mLeftPrimaryMotor = new CANSparkMax(Constants.CAN.SHOOTER_LEFT_PRIMARY, MotorType.kBrushless);
    mRightPrimaryMotor = new CANSparkMax(Constants.CAN.SHOOTER_RIGHT_PRIMARY, MotorType.kBrushless);
    // mLeftSecondaryMotor = new CANSparkMax(1, MotorType.kBrushless);
    // mRightSecondaryMotor = new CANSparkMax(1, MotorType.kBrushless);

    mLeftPrimaryMotor.restoreFactoryDefaults();
    mRightPrimaryMotor.restoreFactoryDefaults();
    // mLeftSecondaryMotor.restoreFactoryDefaults();
    // mRightSecondaryMotor.restoreFactoryDefaults();  

    mLeftPrimaryMotor.setIdleMode(IdleMode.kBrake);
    mRightPrimaryMotor.setIdleMode(IdleMode.kBrake);

    mLeftPrimaryMotor.setInverted(true);
    mRightPrimaryMotor.setInverted(true);
  
    mLeftEncoder = mLeftPrimaryMotor.getEncoder();
    mRightEncoder = mRightPrimaryMotor.getEncoder();

    mLeftPrimaryController = mLeftPrimaryMotor.getPIDController();
    mRightPrimaryController = mRightPrimaryMotor.getPIDController();

    // mLimitSwitch = mLiftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    // mLimitSwitch.enableLimitSwitch(true);

    // mLimitSwitch = new DigitalInput(0);

    mLeftPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mLeftPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, 255); //The lifter maxes at 255 rotations
    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    mLifterControlMode = LifterControlMode.OPEN_LOOP;

    kP = 0;
    kI = 0;
    kD = 0;
    kMinOutput = -0.5;
    kMaxOutput = -0.5;

    configureLifterController();

    SmartDashboard.putNumber("Lift_P", mLeftPrimaryController.getP());
    SmartDashboard.putNumber("Lift_I", mLeftPrimaryController.getI());
    SmartDashboard.putNumber("Lift_D", mLeftPrimaryController.getD());
    SmartDashboard.putNumber("Lift forward limit", mLeftPrimaryMotor.getSoftLimit(SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Lift reverse limit", mLeftPrimaryMotor.getSoftLimit(SoftLimitDirection.kReverse));

    // goHome();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putBoolean("Limit Switch isPressed", mLimitSwitch.get());

    // stopAtHome();
    
  }

  public void setPosition(double setPoint){
    mLeftPrimaryController.setReference(setPoint, ControlType.kPosition);
  }

  public void runLiftWithJoystick(XboxController controller){
    updateFromSmartDashboard();
    
    if (mLifterControlMode == LifterControlMode.POSITION_CONTROL){
      setPosition(controller.getRightY() * 50);
    }
    if (mLifterControlMode == LifterControlMode.OPEN_LOOP){
      mLeftPrimaryMotor.set(controller.getRightY() * 0.2);
    }
  }

  public void resetEncoders(){
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  public double rotationsToMeters(double rotations){
    return 0;
  }

  public void switchLifterMode(){
    if (mLifterControlMode == LifterControlMode.OPEN_LOOP){
      setControlMode(LifterControlMode.POSITION_CONTROL);
      System.out.println("Lifter: switched to Position Control");
    }
    else if(mLifterControlMode == LifterControlMode.POSITION_CONTROL){
      setControlMode(LifterControlMode.OPEN_LOOP);
      System.out.println("Lifter: switched to Open Loop");
    }
  }

  public void updateFromSmartDashboard(){
    double p = SmartDashboard.getNumber("Lift_P", 0);
    double i = SmartDashboard.getNumber("Lift_I", 0);
    double d = SmartDashboard.getNumber("Lift_D", 0);

    if(p != mLeftPrimaryController.getP()) {mLeftPrimaryController.setP(p);}
    if(i != mLeftPrimaryController.getI()) {mLeftPrimaryController.setI(i);}
    if(d != mLeftPrimaryController.getD()) {mLeftPrimaryController.setD(d);}

    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, (float)SmartDashboard.getNumber("Lift forward limit", 255));
    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)SmartDashboard.getNumber("Lift reverse limit", 0));
  }

  public void configureLifterController(){
    mLeftPrimaryController.setP(kP);
    mLeftPrimaryController.setI(kI);
    mLeftPrimaryController.setD(kD);
    mLeftPrimaryController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public LifterControlMode getControlMode(){
    return mLifterControlMode;
  }

  public void setControlMode(LifterControlMode mode){
    mLifterControlMode = mode;
  }

  // public boolean isPressed(){
  //   return !mLimitSwitch.get();
  // }

  // public void goHome(){
  //   if (!isAtHome()){
  //     mLiftMotor.set(-0.2);
  //   }
  //   resetEncoder();
  // }

  // public void stopAtHome(){
  //   if (isPressed()){
  //     mLiftMotor.stopMotor();
  //   }
  // }

  public enum LifterControlMode{
    OPEN_LOOP, POSITION_CONTROL
  }
}
