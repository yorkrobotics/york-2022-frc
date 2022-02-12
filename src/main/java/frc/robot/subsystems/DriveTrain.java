// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  //Hardware setup
  private CANSparkMax mleftFront, mleftBack, mrightFront, mrightBack;
  private RelativeEncoder mleftFrontEncoder, mrightFrontEncoder;
  private DoubleSolenoid mShifter;

  //Controller setup
  private SparkMaxPIDController mleftPIDController, mrightPIDController;
  private DriveControlState mDriveControlState;

  private double gearRatio, kS, kV, kA;
  private double kP_velocity, kI_velocity, kD_velocity, kMinOutput_velocity, kMaxOutput_velocity;
  private double kP_position, kI_position, kD_position, kMinOutput_position, kMaxOutput_position;

  private DifferentialDriveKinematics mKinematics;
  private DifferentialDriveOdometry mOdometry;
  private ADXRS450_Gyro mGyro;
  private Pose2d mPose;
  private SimpleMotorFeedforward mFeedforward;


  /** Creates a new VelocityController. */
  public DriveTrain() {
    //Hardware
    mleftFront = new CANSparkMax(1, MotorType.kBrushless);
    mleftBack = new CANSparkMax(2, MotorType.kBrushless);
    mrightFront = new CANSparkMax(4, MotorType.kBrushless);
    mrightBack = new CANSparkMax(3, MotorType.kBrushless);

    mShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);
    shiftDown();
    gearRatio = Constants.GEAR_RATIO_LOW;

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
    kP_velocity = 0.000097988; 
    kI_velocity = 0;
    kD_velocity = 0; 
    kMinOutput_velocity = -0.7;  
    kMaxOutput_velocity = 0.7; 

    kP_position = 5.7673; 
    kI_position = 0;
    kD_position = 0; 
    kMinOutput_position = -0.5;
    kMaxOutput_position = 0.5; 

    kS = 0.23123;
    kV = 4.5288;
    kA = 0.4136;

    SmartDashboard.putNumber("Velocity P Gain", kP_velocity);
    SmartDashboard.putNumber("Velocity I Gain", kI_velocity);
    SmartDashboard.putNumber("Velocity D Gain", kD_velocity);
    SmartDashboard.putNumber("Velocity Max Output", kMaxOutput_velocity);
    SmartDashboard.putNumber("Velocity Min Output", kMinOutput_velocity);

    // SmartDashboard.putNumber("Position P Gain", kP_position);
    // SmartDashboard.putNumber("Position I Gain", kI_position);
    // SmartDashboard.putNumber("Position D Gain", kD_position);
    // SmartDashboard.putNumber("Position Max Output", kMaxOutput_position);
    // SmartDashboard.putNumber("Position Min Output", kMinOutput_velocity);

    resetEncoders();

    mGyro = new ADXRS450_Gyro();
    mGyro.calibrate();

    mKinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    mOdometry = new DifferentialDriveOdometry(mGyro.getRotation2d()); //optional second arguement: starting position
    mFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    Shuffleboard.getTab("Gyro").add(mGyro);

    var autoTab = Shuffleboard.getTab("Autonomous");
    autoTab.addNumber("Pose X", () -> {
      return this.getPose().getX();
    });
    autoTab.addNumber("Pose Y", () -> {
      return this.getPose().getY();
    });
    autoTab.addNumber("Left Wheel Speeds", () -> {
      return this.getWheelSpeeds().leftMetersPerSecond;
    });
    autoTab.addNumber("Right Wheel Speeds", () -> {
      return this.getWheelSpeeds().rightMetersPerSecond;
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update and get values from the smart dashboard
    if (mDriveControlState == DriveControlState.VELOCITY_CONTROL){
      
      double p = SmartDashboard.getNumber("Velocity P Gain", 0);
      double i = SmartDashboard.getNumber("Velocity I Gain", 0);
      double d = SmartDashboard.getNumber("Velocity D Gain", 0);
      double max = SmartDashboard.getNumber("Velocity Max Output", 0);
      double min = SmartDashboard.getNumber("Velocity Min Output", 0);

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

    mPose = mOdometry.update(mGyro.getRotation2d(), rotationsToMeters(mleftFrontEncoder.getPosition()), rotationsToMeters(mrightFrontEncoder.getPosition()));

    
    SmartDashboard.putNumber("left_position", rotationsToMeters(mleftFrontEncoder.getPosition()));
    SmartDashboard.putNumber("right_position", rotationsToMeters(mleftFrontEncoder.getPosition()));

    SmartDashboard.putNumber("left_velocity", mleftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("right_velocity", mrightFrontEncoder.getVelocity());

    SmartDashboard.putNumber("left_wheelspeed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("right_wheelspeed", getWheelSpeeds().rightMetersPerSecond);



    SmartDashboard.putNumber("heading angle", mGyro.getRate());
    SmartDashboard.putBoolean("Gyro Connected", mGyro.isConnected());

  }

  public void configureVelocityControl(){
    mDriveControlState = DriveControlState.VELOCITY_CONTROL;    

    updatePIDController(mleftPIDController, kP_velocity, kI_velocity, kD_velocity, kMinOutput_velocity, kMaxOutput_velocity);
    updatePIDController(mrightPIDController, kP_velocity, kI_velocity, kD_velocity, kMinOutput_velocity, kMaxOutput_velocity);
  }

  public void configurePositionControl(){
    mDriveControlState = DriveControlState.POSITION_CONTROL;

    updatePIDController(mleftPIDController, kP_position, kI_position, kD_position, kMinOutput_position, kMaxOutput_position);
    updatePIDController(mrightPIDController, kP_position, kI_position, kD_position, kMinOutput_position, kMaxOutput_position);

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
      System.out.println("[Drive] drive mode not in open loop");
    }
  }

  // Setting the motors according to velocity control
  public void setVelocity(double left_velocity, double right_velocity){
    if (mDriveControlState == DriveControlState.VELOCITY_CONTROL){
      mleftPIDController.setReference(left_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
      mrightPIDController.setReference(right_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
    }
    else{
      System.out.println("[Drive] drive mode not in velocity control");
    }
  }

  // Setting the motors according to position control
  public void setPosition(double setMeters){
    if (mDriveControlState == DriveControlState.POSITION_CONTROL){
      double setRotations = metersToRotations(setMeters);
      mleftPIDController.setReference(metersToRotations(setRotations), CANSparkMax.ControlType.kPosition);
      mrightPIDController.setReference(metersToRotations(setRotations), CANSparkMax.ControlType.kPosition);
    }
    else{
      System.out.println("[Drive] drive mode not in position control");
    }
  }

  public void updatePIDController(SparkMaxPIDController pidController, double p, double i, double d, double minOutput, double maxOutput){
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setIZone(0);
    pidController.setFF(1.5e-5);
    pidController.setOutputRange(minOutput, maxOutput);
  }

  public WheelSpeeds mArcadeDrive(XboxController controller){
    // double controller_leftX = controller.getLeftX();
    // if (Math.abs(controller_leftX) < 0.25) controller_leftX = 0;
    return DifferentialDrive.arcadeDriveIK(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller.getLeftX(), true);
    
  }

  public void shiftUp(){
    mShifter.set(Value.kReverse);
    gearRatio = Constants.GEAR_RATIO_HIGH;
  }

  public void shiftDown(){
    mShifter.set(Value.kForward);
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

  public double getHeading(){
    return mGyro.getRotation2d().getDegrees();
  }

  public SimpleMotorFeedforward getFeedForward(){
    return mFeedforward;
  }

  public DifferentialDriveKinematics getKinematics(){
    return mKinematics;
  }

  public DifferentialDriveOdometry getOdometry(){
    return mOdometry;
  }

  public Pose2d getPose(){
    return mPose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      rotationsToMeters(mleftFrontEncoder.getVelocity()) / 60, 
      rotationsToMeters(mrightFrontEncoder.getVelocity()) / 60);
  }

  public void resetEncoders(){
    mleftFrontEncoder.setPosition(0);
    mrightFrontEncoder.setPosition(0);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    mleftFront.setVoltage(leftVolts);
    mrightFront.setVoltage(rightVolts);
  }

  public enum DriveControlState{
    OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL
  } 

}

