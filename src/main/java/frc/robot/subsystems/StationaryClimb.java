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

public class StationaryClimb extends SubsystemBase {

  private static final StationaryClimb mStationaryClimb = new StationaryClimb();
  public static StationaryClimb getInstance() {return mStationaryClimb;}

  private CANSparkMax mLeftMotor, mRightMotor;
  private SparkMaxPIDController mLeftPIDController, mRightPIDController;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  private SparkMaxLimitSwitch mLeftLimitSwitch, mRightLimitSwitch;

  private double mSetpoint = 0;
  
  private StationaryClimbMode mClimbMode;

  private double kP, kI, kD, kMinOutput, kMaxOutput;
  /** Creates a new StationaryClimb. */
  public StationaryClimb() {
    mLeftMotor = new CANSparkMax(Constants.SparkMax.STATIONARY_CLIMB_LEFT, MotorType.kBrushless);
    mRightMotor = new CANSparkMax(Constants.SparkMax.STATIONARY_CLIMB_RIGHT, MotorType.kBrushless); 

    mLeftMotor.setIdleMode(Constants.CLIMB_IDLE_MODE);
    mRightMotor.setIdleMode(Constants.CLIMB_IDLE_MODE);

    mLeftMotor.setInverted(true);
    mRightMotor.setInverted(false);
  
    mLeftEncoder = mLeftMotor.getEncoder();
    mRightEncoder = mRightMotor.getEncoder();

    mLeftPIDController = mLeftMotor.getPIDController();
    mRightPIDController = mRightMotor.getPIDController();

    mLeftLimitSwitch = mLeftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    mRightLimitSwitch = mRightMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    mLeftLimitSwitch.enableLimitSwitch(true);
    mRightLimitSwitch.enableLimitSwitch(true);


    mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
    mRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
    mRightMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);

    //the lifters extends in kReverse direction
    mLeftMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    mLeftMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.STATIONARY_CLIMB_REVERSE_LIMIT);
    mRightMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    mRightMotor.setSoftLimit(SoftLimitDirection.kReverse, Constants.STATIONARY_CLIMB_REVERSE_LIMIT);

    mClimbMode = StationaryClimbMode.OPEN_LOOP;

    kP = 0.1;
    kI = 0;
    kD = 0;
    kMinOutput = -0.5;
    kMaxOutput = 0.5;

    configurePIDController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Stationary Climb Left Position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Stationary Climb Right Position", mRightEncoder.getPosition());

    SmartDashboard.putBoolean("Stationary Climb Left isPressed", mLeftLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Stationary Climb Right isPressed", mRightLimitSwitch.isPressed());
  }

    /**
   * Run motors according to position setpoint
   * @param setPoint setpoint in rotations
   */
  public void runClimbPositionControl(double setPoint){
    setPoint = Double.min(setPoint, 0);
    setPoint = Double.max(setPoint, (double) Constants.STATIONARY_CLIMB_REVERSE_LIMIT);
    mLeftPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    mRightPIDController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Run motors in open loop
   * @param speed speed between (-1, 1)
   */
  public void runClimbOpenLoop(double speed){
    mLeftMotor.set(speed);
    mRightMotor.set(speed);
  }

  /**
   * Run climb motors according to climb mode
   * @param controller Xbox controller
   */
  public void runClimbWithJoystick(XboxController controller){
    // updateFromSmartDashboard();
    
    if (mClimbMode == StationaryClimbMode.POSITION_CONTROL){
      mSetpoint += controller.getRightY() * 1.5;
      mSetpoint = Double.min(mSetpoint, 0);
      mSetpoint = Double.max(mSetpoint, (double)Constants.STATIONARY_CLIMB_REVERSE_LIMIT);
      runClimbPositionControl(mSetpoint);
    }
    if (mClimbMode == StationaryClimbMode.OPEN_LOOP){
      runClimbOpenLoop(-controller.getRightY() * 0.2);
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
    if (mClimbMode == StationaryClimbMode.OPEN_LOOP){
      setControlMode(StationaryClimbMode.POSITION_CONTROL);
      System.out.println("[Climb] Position Control");
    }
    else if(mClimbMode == StationaryClimbMode.POSITION_CONTROL){
      setControlMode(StationaryClimbMode.OPEN_LOOP);
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

    if(p != kP) {mLeftPIDController.setP(p); mRightPIDController.setP(p); kP = p;}
    if(i != kI) {mLeftPIDController.setI(i); mRightPIDController.setI(i); kI = i;}
    if(d != kD) {mLeftPIDController.setD(d); mRightPIDController.setD(d); kD = d;}
  }

  /**
   * Set PID gains and min, max output of motors
   */
  public void configurePIDController(){
    mLeftPIDController.setP(kP);
    mLeftPIDController.setI(kI);
    mLeftPIDController.setD(kD);
    mLeftPIDController.setOutputRange(kMinOutput, kMaxOutput);

    mRightPIDController.setP(kP);
    mRightPIDController.setI(kI);
    mRightPIDController.setD(kD);
    mRightPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /**
   * Get climb control mode
   * @return climb control mode
   */
  public StationaryClimbMode getControlMode(){
    return mClimbMode;
  }

  /**
   * Set climb control mode
   * @param mode climb control mode to set
   */
  public void setControlMode(StationaryClimbMode mode){
    mClimbMode = mode;
  }


  public void goHome(){
      if (!mLeftLimitSwitch.isPressed()){
        mLeftMotor.set(0.4);
      }
      if (!mRightLimitSwitch.isPressed()){
        mRightMotor.set(0.4);
      }
    
    mSetpoint = 0;
    resetEncoders();
  }

  public boolean isHome(){
    return mLeftLimitSwitch.isPressed() && mRightLimitSwitch.isPressed();
  }

  public void ClimbUpWithBumper(){
    mSetpoint -= 1.8;
    runClimbPositionControl(mSetpoint);
  }

  public void ClimbDownWithBumper(){
    mSetpoint += 1.8;
    runClimbPositionControl(mSetpoint);
  }

  public void zeroSetpoint(){
    mSetpoint = 0;
  }

  public void stopMotors(){
    mLeftMotor.stopMotor();
    mRightMotor.stopMotor();
  }

  private enum StationaryClimbMode{
    OPEN_LOOP, POSITION_CONTROL
  }
}
