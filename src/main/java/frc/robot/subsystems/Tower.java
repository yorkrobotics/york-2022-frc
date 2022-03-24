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

public class Tower extends SubsystemBase {

  private static final Tower mTower = new Tower();
  public static Tower getInstance(){
    return mTower;
  }

  private CANSparkMax mLeftMotor, mRightMotor;
  private SparkMaxPIDController mLeftController, mRightController;
  private RelativeEncoder mLeftEncoder, mRightEncoder;
  private TowerActuatorMode mActuatorMode;

  private SparkMaxLimitSwitch mRightLimitSwitch, mLeftLimitSwitch;

  private double kP, kI, kD, kMinOutput, kMaxOutput;
  private double x_field;

  /** Creates a new Tower. */
  public Tower() {
    mLeftMotor = new CANSparkMax(Constants.SparkMax.TOWER_ACTUATOR_LEFT, MotorType.kBrushless);
    mRightMotor = new CANSparkMax(Constants.SparkMax.TOWER_ACTUATOR_RIGHT, MotorType.kBrushless);

    mLeftMotor.restoreFactoryDefaults();
    mRightMotor.restoreFactoryDefaults();

    mLeftMotor.setInverted(true);
    mRightMotor.setInverted(true);

    mLeftMotor.setIdleMode(Constants.TOWER_IDLE_MODE);
    mRightMotor.setIdleMode(Constants.TOWER_IDLE_MODE);

    mLeftController = mLeftMotor.getPIDController();
    mRightController = mRightMotor.getPIDController();

    mLeftEncoder = mLeftMotor.getEncoder();
    mRightEncoder = mRightMotor.getEncoder();

    mLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    mRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

    mLeftMotor.setSoftLimit(SoftLimitDirection.kForward, 260);
    mLeftMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    mRightMotor.setSoftLimit(SoftLimitDirection.kForward, 260);
    mRightMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    mLeftLimitSwitch = mLeftMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    mRightLimitSwitch = mRightMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    mLeftLimitSwitch.enableLimitSwitch(true);
    mRightLimitSwitch.enableLimitSwitch(true);

    mActuatorMode = TowerActuatorMode.OPEN_LOOP;

    kP = 0.1;
    kI = 0;
    kD = 0;
    kMinOutput = -0.5;
    kMaxOutput = 0.5;

    configurePIDController();

    SmartDashboard.putNumber("Actuator P", kP);
    SmartDashboard.putNumber("Actuator I", kI);
    SmartDashboard.putNumber("Actuator D", kD);
    SmartDashboard.putNumber("Actuator setpoint", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tower left position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Tower right position", mRightEncoder.getPosition());

    SmartDashboard.putBoolean("Tower Left isPressed", mLeftLimitSwitch.isPressed());
    SmartDashboard.putBoolean("Tower Right isPressed", mRightLimitSwitch.isPressed());
    SmartDashboard.putNumber("Tower angle", this.getTowerAngle());
    SmartDashboard.putNumber("Actuator length", this.rotationToLength());
    x_field = SmartDashboard.getNumber("Field X", 0);
  }

  /**
   * Run motors in open loop
   * @param speed speed between (-1, 1)
   */
  public void runActuatorsOpenLoop(Double speed){
    mLeftMotor.set(speed);
    mRightMotor.set(speed); 
  }

  /**
   * Run motors to position setpoint
   * @param setpoint setpoint in rotations
   */
  public void runActuatorsPositionControl(double setpoint){
    setpoint = Double.min(setpoint, 260);
    setpoint = Double.max(setpoint, 0);
    mLeftController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    mRightController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Run the tower actuators with controller
   * @param controller Xbox controller
   */
  public void runTowerWithController(XboxController controller){
    updateFromSmartDashboard();
    if (mActuatorMode == TowerActuatorMode.POSITION_CONTROL){
      Double setpoint = SmartDashboard.getNumber("Actuator setpoint", 0);
      runActuatorsPositionControl(setpoint);
    }
    if (mActuatorMode == TowerActuatorMode.OPEN_LOOP){
      runActuatorsOpenLoop(controller.getRightX() * 0.2);
    }
  }

  public void goHome(){
    while (!mLeftLimitSwitch.isPressed() || !mRightLimitSwitch.isPressed()){
      if(!mLeftLimitSwitch.isPressed()){
        mLeftMotor.set(-0.2);
      }
      if (!mRightLimitSwitch.isPressed()){
        mRightMotor.set(-0.2);
      }
    }
    resetEncoders();
  }

  public void resetEncoders(){
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  /**
   * Switches actuator mode
   */
  public void switchActuatorMode(){
    if (mActuatorMode == TowerActuatorMode.POSITION_CONTROL){
      mActuatorMode = TowerActuatorMode.OPEN_LOOP;
      System.out.println("[Tower] Open loop");
    }
    else if (mActuatorMode == TowerActuatorMode.OPEN_LOOP){
      mActuatorMode = TowerActuatorMode.POSITION_CONTROL;
      System.out.println("[Tower] position control");
    }
  }

  /**
   * Set PID gains and min, max output of motors
   */
  public void configurePIDController(){
    mLeftController.setP(kP);
    mLeftController.setI(kI);
    mLeftController.setD(kD);
    mLeftController.setOutputRange(kMinOutput, kMaxOutput);

    mRightController.setP(kP);
    mRightController.setI(kI);
    mRightController.setD(kD);
    mRightController.setOutputRange(kMinOutput, kMaxOutput);
  }

  /**
   * Get and update values from SmartDashboard
   */
  public void updateFromSmartDashboard(){
    double p = SmartDashboard.getNumber("Climb_P", 0);
    double i = SmartDashboard.getNumber("Climb_I", 0);
    double d = SmartDashboard.getNumber("Climb_D", 0);

    if(p != mLeftController.getP()) {mLeftController.setP(p); mRightController.setP(p); kP = p;}
    if(i != mLeftController.getI()) {mLeftController.setI(i); mRightController.setI(i); kI = i;}
    if(d != mLeftController.getD()) {mLeftController.setD(d); mRightController.setD(d); kD = d;}
  }

  // returns the length of the actuator using the encoder position
  public double rotationToLength() {
    double ratio = 1.7 / 100; // ratio of inch to every rotation (100 rotations per 1.75 inc)
    double base = 8.8; // the extra part that doesn't extend
    double actuatorLength = mRightEncoder.getPosition() * ratio + base;
    return actuatorLength;
  }

  public double LengthToRotation(double actuatorLength) {
    double ratio = 100 /1.7;
    double base = 8.8;
    double rotations = (actuatorLength - base) * ratio;
    return rotations;
  }

  // returns the current angle of the tower in degrees
  public double getTowerAngle() {
    double radius = 12;
    double baseLength = 9.5;
    double actuatorLength = this.rotationToLength();
    double vOffset = 5 / 180 * Math.PI; // vertical offset from the ground in radian
    double towerAngle = Math.acos((Math.pow(radius, 2) + Math.pow(baseLength, 2) - Math.pow(actuatorLength, 2))/(2 * baseLength *  radius)) - vOffset;
    return towerAngle / Math.PI * 180;
  }

  public void setTowerAngle(double angle) {
    double radius = 12;
    double baseLength = 9.5;
    double vOffset = 5 / 180 * Math.PI; // vertical offset from the ground in radian

    double acutatorLength =  Math.sqrt(Math.pow(radius, 2)+ Math.pow(baseLength, 2) - 2 * radius * baseLength * Math.cos(angle / 180 * Math.PI + vOffset));

    double encoderPos = LengthToRotation(acutatorLength);
    this.runActuatorsPositionControl(encoderPos);
  }

  public void aimTarget()  {
    double targetAngle = 67.6818 - 0.101674* x_field;
    this.setTowerAngle(targetAngle);
  }


  private enum TowerActuatorMode{
    POSITION_CONTROL, OPEN_LOOP, OFF
  }
}
