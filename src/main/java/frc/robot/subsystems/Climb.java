// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
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
  private SparkMaxLimitSwitch mLeftLimitSwitch, mRightLimitSwitch;

  private double mSetpoint = 0;
  
  private ClimbMode mClimbMode;

  private double kP, kI, kD, kMinOutput, kMaxOutput;

  public Climb() {
    mLeftPrimaryMotor = new CANSparkMax(Constants.SparkMax.CLIMB_LEFT_PRIMARY, MotorType.kBrushless);
    mRightPrimaryMotor = new CANSparkMax(Constants.SparkMax.CLIMB_RIGHT_PRIMARY, MotorType.kBrushless); 

    mLeftPrimaryMotor.setIdleMode(Constants.CLIMB_IDLE_MODE);
    mRightPrimaryMotor.setIdleMode(Constants.CLIMB_IDLE_MODE);

    mLeftPrimaryMotor.setInverted(true);
    mRightPrimaryMotor.setInverted(false);
  
    mLeftEncoder = mLeftPrimaryMotor.getEncoder();
    mRightEncoder = mRightPrimaryMotor.getEncoder();

    mLeftPrimaryController = mLeftPrimaryMotor.getPIDController();
    mRightPrimaryController = mRightPrimaryMotor.getPIDController();

    mLeftLimitSwitch = mLeftPrimaryMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    mRightLimitSwitch = mRightPrimaryMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    mLeftLimitSwitch.enableLimitSwitch(true);
    mRightLimitSwitch.enableLimitSwitch(true);


    mLeftPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    mLeftPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    mRightPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    mRightPrimaryMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    //the lifters extends in kReverse direction
    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    mLeftPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.CLIMB_REVERSE_LIMIT);
    mRightPrimaryMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    mRightPrimaryMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.CLIMB_REVERSE_LIMIT);

    mClimbMode = ClimbMode.POSITION_CONTROL;

    kP = 0.1;
    kI = 0;
    kD = 0;
    kMinOutput = -0.5;
    kMaxOutput = 0.5;

    configurePIDController();

    SmartDashboard.putNumber("Climb_P", kP);
    SmartDashboard.putNumber("Climb_I", kI);
    SmartDashboard.putNumber("Climb_D", kD);
    SmartDashboard.putNumber("Climb setpoint", mSetpoint);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // SmartDashboard.putBoolean("Limit Switch isPressed", mLimitSwitch.get());

    // stopAtHome();

    SmartDashboard.putNumber("Climb left position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Climb right position", mRightEncoder.getPosition());
    
    SmartDashboard.putBoolean("Lift Left isPressed", mLeftLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Lift Right isPressed", mRightLimitSwitch.isPressed());
  }

  /**
   * Run motors according to position setpoint
   * @param setPoint setpoint in rotations
   */
  public void runClimbPositionControl(double setPoint){
    setPoint = Double.min(setPoint, 0);
    setPoint = Double.max(setPoint, (double) Constants.CLIMB_REVERSE_LIMIT);
    mLeftPrimaryController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    mRightPrimaryController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Run motors in open loop
   * @param speed speed between (-1, 1)
   */
  public void runClimbOpenLoop(double speed){
    mLeftPrimaryMotor.set(speed);
    mRightPrimaryMotor.set(speed);
  }

  /**
   * Run climb motors according to climb mode
   * @param controller Xbox controller
   */
  public void runClimbWithJoystick(XboxController controller){
    updateFromSmartDashboard();
    
    if (mClimbMode == ClimbMode.POSITION_CONTROL){
      mSetpoint += controller.getRightY() * 1.5;
      mSetpoint = Double.min(mSetpoint, 0);
      mSetpoint = Double.max(mSetpoint, (double)Constants.CLIMB_REVERSE_LIMIT);
      runClimbPositionControl(mSetpoint);
    }
    if (mClimbMode == ClimbMode.OPEN_LOOP){
      runClimbOpenLoop(controller.getRightY() * 0.6);
    }
  }

  /**
   * Reset left and right encoders
   */
  public void resetEncoders(){
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  /**
   * Switch climb mode between open loop and position control
   */
  public void switchClimbMode(){
    if (mClimbMode == ClimbMode.OPEN_LOOP){
      setControlMode(ClimbMode.POSITION_CONTROL);
      System.out.println("[Climb] Position Control");
    }
    else if(mClimbMode == ClimbMode.POSITION_CONTROL){
      setControlMode(ClimbMode.OPEN_LOOP);
      System.out.println("[Climb] Open Loop");
    }
  }

  /**
   * Get and update values from SmartDashboard
   */
  public void updateFromSmartDashboard(){
    double p = SmartDashboard.getNumber("Climb_P", 0);
    double i = SmartDashboard.getNumber("Climb_I", 0);
    double d = SmartDashboard.getNumber("Climb_D", 0);

    if(p != kP) {mLeftPrimaryController.setP(p); mRightPrimaryController.setP(p); kP = p;}
    if(i != kI) {mLeftPrimaryController.setI(i); mRightPrimaryController.setI(i); kI = i;}
    if(d != kD) {mLeftPrimaryController.setD(d); mRightPrimaryController.setD(d); kD = d;}
  }

  /**
   * Set PID gains and min, max output of motors
   */
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

  /**
   * Get climb control mode
   * @return climb control mode
   */
  public ClimbMode getControlMode(){
    return mClimbMode;
  }

  /**
   * Set climb control mode
   * @param mode climb control mode to set
   */
  public void setControlMode(ClimbMode mode){
    mClimbMode = mode;
  }


  public void goHome(){
    while (!mLeftLimitSwitch.isPressed() || !mRightLimitSwitch.isPressed()){
      if (!mLeftLimitSwitch.isPressed()){
        mLeftPrimaryMotor.set(0.4);
      }
      if (!mRightLimitSwitch.isPressed()){
        mRightPrimaryMotor.set(0.4);
      }
    }
    mSetpoint = 0;
    resetEncoders();
  }

  public boolean isHome(){
    return mLeftLimitSwitch.isPressed() && mRightLimitSwitch.isPressed();
  }

  public void ClimbUpWithBumper(){
    mSetpoint -= 1.5;
    runClimbPositionControl(mSetpoint);
  }

  public void ClimbDownWithBumper(){
    mSetpoint += 1.5;
    runClimbPositionControl(mSetpoint);
  }

  public void stopMotors(){
    mLeftPrimaryMotor.stopMotor();
    mRightPrimaryMotor.stopMotor();
  }

  public enum ClimbMode{
    OPEN_LOOP, POSITION_CONTROL
  }
}
