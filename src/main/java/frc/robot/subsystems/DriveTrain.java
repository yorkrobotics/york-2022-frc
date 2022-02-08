// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  //Hardware setup
  private CANSparkMax mleftFront, mleftBack, mrightFront, mrightBack;
  private RelativeEncoder mleftFrontEncoder, mrightFrontEncoder;
  //Controller setup
  private SparkMaxPIDController mleftPIDController, mrightPIDController;
  private DriveControlState mDriveControlState;

  private double gearRatio;
  private double kP_velocity, kI_velocity, kD_velocity, kMinOutput_velocity, kMaxOutput_velocity;
  private double kP_position, kI_position, kD_position, kMinOutput_position, kMaxOutput_position;


  /** Creates a new VelocityController. */
  public DriveTrain() {
    //Hardware
    mleftFront = new CANSparkMax(1, MotorType.kBrushless);
    mleftBack = new CANSparkMax(2, MotorType.kBrushless);
    mrightFront = new CANSparkMax(4, MotorType.kBrushless);
    mrightBack = new CANSparkMax(3, MotorType.kBrushless);

    mleftFront.restoreFactoryDefaults();
    mleftBack.restoreFactoryDefaults();
    mrightFront.restoreFactoryDefaults();
    mrightBack.restoreFactoryDefaults();

    mleftBack.follow(mleftFront);
    mrightBack.follow(mrightFront);

    mleftFront.setInverted(false);
    mrightFront.setInverted(true);

    mleftFrontEncoder = mleftFront.getEncoder();
    mrightFrontEncoder = mrightFront.getEncoder();
    
    //Controller
    mleftPIDController = mleftFront.getPIDController();
    mrightPIDController = mrightFront.getPIDController();

    mDriveControlState = DriveControlState.OPEN_LOOP; //Default drive mode set to open loop

    //Store PID coefficients
    kP_velocity = 6e-6; 
    kI_velocity = 5e-7;
    kD_velocity = 1e-6; 
    kMinOutput_velocity = -0.5;  
    kMaxOutput_velocity = 0.5; 

    kP_position = 0; 
    kI_position = 0;
    kD_position = 0; 
    kMinOutput_position = -0.5;
    kMaxOutput_position = 0.5; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update and get values from the smart dashboard
    if (mDriveControlState == DriveControlState.VELOCITY_CONTROL){
      SmartDashboard.putNumber("P Gain", kP_velocity);
      SmartDashboard.putNumber("I Gain", kI_velocity);
      SmartDashboard.putNumber("D Gain", kD_velocity);
      SmartDashboard.putNumber("Max Output", kMaxOutput_velocity);
      SmartDashboard.putNumber("Min Output", kMinOutput_velocity);

      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);

      if((p != kP_velocity)) { mleftPIDController.setP(p); mrightPIDController.setP(p); kP_velocity = p; configureVelocityControl();}
      if((i != kI_velocity)) { mleftPIDController.setI(i); mrightPIDController.setI(i); kI_velocity = i; configureVelocityControl();}
      if((d != kD_velocity)) { mleftPIDController.setD(d); mrightPIDController.setD(d); kD_velocity = d; configureVelocityControl();}
      if((max != kMaxOutput_velocity) || (min != kMinOutput_velocity)) { 
        mleftPIDController.setOutputRange(min, max); 
        mrightPIDController.setOutputRange(min, max);
        kMinOutput_velocity = min; kMaxOutput_velocity = max; 
        configureVelocityControl();
      }
    }
    
    SmartDashboard.putNumber("left_encoder", rotationsToMeters(mleftFrontEncoder.getPosition()));
    SmartDashboard.putNumber("right_encoder", rotationsToMeters(mleftFrontEncoder.getPosition()));
  }

  public void configureVelocityControl(){
    mDriveControlState = DriveControlState.VELOCITY_CONTROL;    

    initializePIDController(mleftPIDController, kP_velocity, kI_velocity, kD_velocity, kMinOutput_velocity, kMaxOutput_velocity);
    initializePIDController(mrightPIDController, kP_velocity, kI_velocity, kD_velocity, kMinOutput_velocity, kMaxOutput_velocity);
  }

  public void configurePositionControl(){
    mDriveControlState = DriveControlState.POSITION_CONTROL;

    initializePIDController(mleftPIDController, kP_position, kI_position, kD_position, kMinOutput_position, kMaxOutput_position);
    initializePIDController(mrightPIDController, kP_position, kI_position, kD_position, kMinOutput_position, kMaxOutput_position);

    //extra step to reset the encoders
    mleftFrontEncoder.setPosition(0);
    mrightFrontEncoder.setPosition(0);
  }

  public void setOpenLoop(double left_velocity, double right_velocity){
    if (mDriveControlState == DriveControlState.OPEN_LOOP){
      mleftFront.set(left_velocity * Constants.MAX_OPENLOOP_SPEED);
      mrightFront.set(right_velocity * Constants.MAX_OPENLOOP_SPEED);
    }
    else{
      System.out.println("drive mode not in open loop");
    }
  }

  // Setting the motors according to velocity control
  public void setVelocity(double left_velocity, double right_velocity){
    if (mDriveControlState == DriveControlState.VELOCITY_CONTROL){
      mleftPIDController.setReference(left_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
      mrightPIDController.setReference(right_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
    }
    else{
      System.out.println("drive mode not in velocity control");
    }
  }

  // Setting the motors according to position control
  public void setPosition(double setPoint){
    if (mDriveControlState == DriveControlState.POSITION_CONTROL){
      mleftPIDController.setReference(metersToRotations(setPoint), CANSparkMax.ControlType.kPosition);
      mrightPIDController.setReference(metersToRotations(setPoint), CANSparkMax.ControlType.kPosition);
    }
    else{
      System.out.println("drive mode not in position control");
    }
  }

  public void initializePIDController(SparkMaxPIDController pidController, double p, double i, double d, double minOutput, double maxOutput){
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setIZone(0);
    pidController.setFF(1.5e-5);
    pidController.setOutputRange(minOutput, maxOutput);
  }


  public double[] mArcadeDrive(XboxController controller){
    double xSpeed = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    double zRotaion = controller.getLeftX();
    //Bound joystick input due to crappy controller
    if (Math.abs(zRotaion) < 0.25) zRotaion = 0;
  
    SmartDashboard.putNumber("Controller LeftX", controller.getLeftX());
    SmartDashboard.putNumber("zRotation", zRotaion);

    xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    zRotaion = MathUtil.clamp(zRotaion, -1.0, 1.0);
    double leftSpeed;
    double rightSpeed;
    double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotaion)), xSpeed);
    if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotaion >= 0.0) {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotaion;
      } else {
        leftSpeed = xSpeed + zRotaion;
        rightSpeed = maxInput;
      }
    } else {
      // Third quadrant, else fourth quadrant
      if (zRotaion >= 0.0) {
        leftSpeed = xSpeed + zRotaion;
        rightSpeed = maxInput;
      } else {
        leftSpeed = maxInput;
        rightSpeed = xSpeed - zRotaion;
      }
    }
    // Normalize the wheelspeeds
    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxMagnitude > 1.0) {
      leftSpeed /= maxMagnitude;
      rightSpeed /= maxMagnitude;
    }
    SmartDashboard.putNumber("ArcadeDrive_leftSpeed", leftSpeed);
    SmartDashboard.putNumber("ArcadeDrive_rightSpeed", rightSpeed);

    double[] wheelspeeds = {leftSpeed, rightSpeed};
    return wheelspeeds;
  }

  public WheelSpeeds getWheelSpeeds(XboxController controller){
    // double controller_leftX = controller.getLeftX();
    // if (Math.abs(controller_leftX) < 0.25) controller_leftX = 0;
    return DifferentialDrive.arcadeDriveIK(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller.getLeftX(), true);
    
  }

  public void setToHighGear(){
    gearRatio = Constants.GEAR_RATIO_HIGH;
  }

  public void setToLowGear(){
    gearRatio = Constants.GEAR_RATIO_LOW;
  }

  public double rotationsToMeters(double rotations){
    return rotations / gearRatio * 0.145 * Math.PI;
  }

  public double metersToRotations(double meters){
    return meters * gearRatio / 0.145 / Math.PI;
  }

  public DriveControlState getDriveControlState(){
    return mDriveControlState;
  }

  public void setToVelocityControl(){
    mDriveControlState = DriveControlState.VELOCITY_CONTROL;
  }

  public void setToPositionControl(){
    mDriveControlState = DriveControlState.POSITION_CONTROL;
  }

  public void setOpenLoop(){
    mDriveControlState = DriveControlState.OPEN_LOOP;
  }

  public boolean isAtSetpoint(double setpoint){
    return rotationsToMeters(mleftFrontEncoder.getPosition()) == setpoint;
  }

  public enum DriveControlState{
    OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL
  } 

}

