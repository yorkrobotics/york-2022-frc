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

public class Climb extends SubsystemBase {
  private static final Climb mClimb = new Climb();
  public static Climb getInstance(){
    return mClimb;
  }

  private CANSparkMax mLeftPrimaryMotor, mRightPrimaryMotor;
  private SparkMaxPIDController mLeftPrimaryController, mRightPrimaryController;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  // private SparkMaxLimitSwitch mLimitSwitch;
  // private DigitalInput mLimitSwitch;

  
  private ClimbMode mClimbMode;

  private double kP, kI, kD, kMinOutput, kMaxOutput;

  public Climb() {
    mLeftPrimaryMotor = new CANSparkMax(Constants.CAN.CLIMB_LEFT_PRIMARY, MotorType.kBrushless);
    mRightPrimaryMotor = new CANSparkMax(Constants.CAN.CLIMB_RIGHT_PRIMARY, MotorType.kBrushless);
    // mLeftSecondaryMotor = new CANSparkMax(1, MotorType.kBrushless);
    // mRightSecondaryMotor = new CANSparkMax(1, MotorType.kBrushless);

    mLeftPrimaryMotor.restoreFactoryDefaults();
    mRightPrimaryMotor.restoreFactoryDefaults();
    // mLeftSecondaryMotor.restoreFactoryDefaults();
    // mRightSecondaryMotor.restoreFactoryDefaults();  

    mLeftPrimaryMotor.setIdleMode(IdleMode.kBrake);
    mRightPrimaryMotor.setIdleMode(IdleMode.kBrake);

    mLeftPrimaryMotor.setInverted(true);
    mRightPrimaryMotor.setInverted(false);
  
    mLeftEncoder = mLeftPrimaryMotor.getEncoder();
    mRightEncoder = mRightPrimaryMotor.getEncoder();

    mLeftPrimaryController = mLeftPrimaryMotor.getPIDController();
    mRightPrimaryController = mRightPrimaryMotor.getPIDController();

    // mLimitSwitch = mLiftMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    // mLimitSwitch.enableLimitSwitch(true);

    // mLimitSwitch = new DigitalInput(0);

    mLeftPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mLeftPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    mRightPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    mRightPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, 0); //The lifter maxes at 255 rotations
    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, -255);
    mRightPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, -255);
    mRightPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    mClimbMode = ClimbMode.OPEN_LOOP;

    kP = 0;
    kI = 0;
    kD = 0;
    kMinOutput = -0.5;
    kMaxOutput = -0.5;

    configurePIDController();

    SmartDashboard.putNumber("Climb_P", mLeftPrimaryController.getP());
    SmartDashboard.putNumber("Climb_I", mLeftPrimaryController.getI());
    SmartDashboard.putNumber("Climb_D", mLeftPrimaryController.getD());
    SmartDashboard.putNumber("Climb forward limit", mLeftPrimaryMotor.getSoftLimit(SoftLimitDirection.kForward));
    SmartDashboard.putNumber("Climb reverse limit", mLeftPrimaryMotor.getSoftLimit(SoftLimitDirection.kReverse));

    // goHome();


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putBoolean("Limit Switch isPressed", mLimitSwitch.get());

    // stopAtHome();

    SmartDashboard.putNumber("Climb left position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Climb right position", mRightEncoder.getPosition());
    
  }

  public void runClimbPositionControl(double setPoint){
    mLeftPrimaryController.setReference(setPoint, ControlType.kPosition);
    mRightPrimaryController.setReference(setPoint, ControlType.kPosition);
  }

  public void runClimbWithJoystick(XboxController controller){
    updateFromSmartDashboard();
    
    if (mClimbMode == ClimbMode.POSITION_CONTROL){
      runClimbPositionControl(controller.getRightY() * 50);
    }
    if (mClimbMode == ClimbMode.OPEN_LOOP){
      mLeftPrimaryMotor.set(controller.getRightY() * 0.2);
      // mRightPrimaryMotor.set(controller.getRightY() * 0.2);
    }
  }

  public void resetEncoders(){
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  public double rotationsToMeters(double rotations){
    return 0;
  }

  public void switchClimbMode(){
    if (mClimbMode == ClimbMode.OPEN_LOOP){
      setControlMode(ClimbMode.POSITION_CONTROL);
      System.out.println("Climb: switched to Position Control");
    }
    else if(mClimbMode == ClimbMode.POSITION_CONTROL){
      setControlMode(ClimbMode.OPEN_LOOP);
      System.out.println("Climb: switched to Open Loop");
    }
  }

  public void updateFromSmartDashboard(){
    double p = SmartDashboard.getNumber("Climb_P", 0);
    double i = SmartDashboard.getNumber("Climb_I", 0);
    double d = SmartDashboard.getNumber("Climb_D", 0);

    if(p != mLeftPrimaryController.getP()) {mLeftPrimaryController.setP(p);}
    if(i != mLeftPrimaryController.getI()) {mLeftPrimaryController.setI(i);}
    if(d != mLeftPrimaryController.getD()) {mLeftPrimaryController.setD(d);}

    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, (float)SmartDashboard.getNumber("Climb forward limit", 255));
    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)SmartDashboard.getNumber("Climb reverse limit", 0));
  }

  public void configurePIDController(){
    mLeftPrimaryController.setP(kP);
    mLeftPrimaryController.setI(kI);
    mLeftPrimaryController.setD(kD);
    mLeftPrimaryController.setOutputRange(kMinOutput, kMaxOutput);

    mRightPrimaryController.setP(kP);
    mRightPrimaryController.setI(kI);
    mRightPrimaryController.setD(kD);
    mRightPrimaryController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public ClimbMode getControlMode(){
    return mClimbMode;
  }

  public void setControlMode(ClimbMode mode){
    mClimbMode = mode;
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

  public enum ClimbMode{
    OPEN_LOOP, POSITION_CONTROL
  }
}
