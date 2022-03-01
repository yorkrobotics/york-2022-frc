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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  //Hardware setup
  private CANSparkMax mleftFront, mleftBack, mrightFront, mrightBack;
  private RelativeEncoder mleftFrontEncoder, mrightFrontEncoder;
  private DoubleSolenoid mShifter;

  //Controller setup
  private SparkMaxPIDController mleftPIDController, mrightPIDController;
  private DriveControlMode mDriveControlMode;
  private GearMode mGearMode;

  private double gearRatio, kS, kV, kA;
  private double kP, kI, kD, kMinOutput, kMaxOutput;

  private DifferentialDriveKinematics mKinematics;
  private DifferentialDriveOdometry mOdometry;
  private ADXRS450_Gyro mGyro;
  private Pose2d mPose;
  private final Field2d mField = new Field2d();
  private SimpleMotorFeedforward mFeedforward;
  private double leftWheelDistance = 0;
  private double rightWheelDistance = 0;
  private double lastLeftEncoderPos = 0;
  private double lastRightEncoderPos = 0;
  private double diffLeft, diffRight, leftMeters, rightMeters;


  /** Creates a new VelocityController. */
  public DriveTrain() {
    //Hardware
    mleftFront = new CANSparkMax(1, MotorType.kBrushless);
    mleftBack = new CANSparkMax(2, MotorType.kBrushless);
    mrightFront = new CANSparkMax(4, MotorType.kBrushless);
    mrightBack = new CANSparkMax(3, MotorType.kBrushless);
    mShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);

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

    setDriveMode(DriveControlMode.OPEN_LOOP); //Default drive mode set to open loop
    shiftDown(); // Always set to low gear at the start

    kS = 0.23123;
    kV = 4.5288;
    kA = 0.4136;

    SmartDashboard.putData("Field", mField);

    resetEncoders();

    mGyro = new ADXRS450_Gyro();
    mGyro.calibrate();

    mKinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    mOdometry = new DifferentialDriveOdometry(new Rotation2d(Constants.STARTING_ANGLE), new Pose2d(Constants.STARTING_X, Constants.STARTING_Y, new Rotation2d(Constants.STARTING_ANGLE))); //optional second arguement: starting position
    mFeedforward = new SimpleMotorFeedforward(kS, kV, kA);

    // Update to Shuffleboard
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
    
    diffLeft = mleftFrontEncoder.getPosition() - lastLeftEncoderPos;
    diffRight = mrightFrontEncoder.getPosition() - lastRightEncoderPos;
    
    leftMeters = rotationsToMeters(diffLeft);
    rightMeters = rotationsToMeters(diffRight);
    
    leftWheelDistance += leftMeters;
    rightWheelDistance += rightMeters;
    
    mPose = mOdometry.update(mGyro.getRotation2d(), leftWheelDistance, rightWheelDistance);
    mField.setRobotPose(mOdometry.getPoseMeters());
    
    lastLeftEncoderPos = mleftFrontEncoder.getPosition();
    lastRightEncoderPos = mrightFrontEncoder.getPosition();
    
    SmartDashboard.putNumber("left_position", rotationsToMeters(mleftFrontEncoder.getPosition()));
    SmartDashboard.putNumber("right_position", rotationsToMeters(mleftFrontEncoder.getPosition()));
    SmartDashboard.putNumber("left_velocity", mleftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("right_velocity", mrightFrontEncoder.getVelocity());

    SmartDashboard.putNumber("left_wheelspeed", getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("right_wheelspeed", getWheelSpeeds().rightMetersPerSecond);

    SmartDashboard.putNumber("heading angle", mGyro.getAngle ());
    SmartDashboard.putBoolean("Gyro Connected", mGyro.isConnected());

    SmartDashboard.putNumber("current heading angle", getHeading());
    // SmartDashboard.putNumber("left_setpoint", mleftPIDController* Constants.DRIVE_MAX_RPM);
    // SmartDashboard.putNumber("right_setpoint", mWheelSpeeds.right * Constants.DRIVE_MAX_RPM);
  }

  public void configureVelocityControl(){
    setDriveMode(DriveControlMode.VELOCITY_CONTROL);
    if (mGearMode == GearMode.LOW_GEAR){
      kP = 0.000097988;
      kI = 0;
      kD = 0;
      kMinOutput = -0.7;
      kMaxOutput = 0.7;
    }
    if (mGearMode == GearMode.HIGH_GEAR){
      kP = 0.000097988;
      kI = 0;
      kD = 0;
      kMinOutput = -0.7;
      kMaxOutput = 0.7;
    }
    updatePIDController(mleftPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    updatePIDController(mrightPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
  }

  public void configurePositionControl(){
    setDriveMode(DriveControlMode.POSITION_CONTROL);
    if (mGearMode == GearMode.LOW_GEAR){
      kP = 0.3;
      kI = 0;
      kD = 0;
      kMinOutput = -0.5;
      kMaxOutput = 0.5;
    }
    if (mGearMode == GearMode.HIGH_GEAR){
      kP = 0.3;
      kI = 0;
      kD = 0;
      kMinOutput = -0.5;
      kMaxOutput = 0.5;
    }
    updatePIDController(mleftPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    updatePIDController(mrightPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    //extra step to reset the encoders
    // resetEncoders();
  }

  public void setOpenLoop(double left_velocity, double right_velocity){
    if (mDriveControlMode == DriveControlMode.OPEN_LOOP){
      mleftFront.set(left_velocity * Constants.MAX_OPENLOOP_SPEED);
      mrightFront.set(right_velocity * Constants.MAX_OPENLOOP_SPEED);
    }
    else{
      System.out.println("[Drive] drive mode not in open loop");
    }
  }

  // Setting the motors according to velocity control
  public void setVelocity(double left_velocity, double right_velocity){
    if (mDriveControlMode == DriveControlMode.VELOCITY_CONTROL){
      mleftPIDController.setReference(left_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
      mrightPIDController.setReference(right_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
    }
    else{
      System.out.println("[Drive] drive mode not in velocity control");
    }
  }

  // Setting the motors according to position control
  public void setPosition(double setMeters){
    configurePositionControl();
    double setRotations = metersToRotations(setMeters);
    mleftPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
    mrightPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
  }

  public void setRotation(double setAngle){
    configurePositionControl();
    mleftPIDController.setOutputRange(-0.5, 0.5);
    mleftPIDController.setP(.3);
    mrightPIDController.setOutputRange(-0.5, 0.5);
    mrightPIDController.setP(.3);

    mleftPIDController.setD(10);
    mrightPIDController.setD(10);

    SmartDashboard.putNumber("set angle", setAngle);
    if (setAngle != 0 ) {
      double setMeters = setAngle / 360 * Math.PI * Constants.TRACK_WIDTH;
      mleftPIDController.setReference(mleftFrontEncoder.getPosition() + metersToRotations(setMeters), CANSparkMax.ControlType.kPosition);
      mrightPIDController.setReference(mrightFrontEncoder.getPosition() - metersToRotations(setMeters), CANSparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("set reference left", mleftFrontEncoder.getPosition() + metersToRotations(setMeters));
      SmartDashboard.putNumber("set reference right", mrightFrontEncoder.getPosition() - metersToRotations(setMeters));
    } else {
      mleftPIDController.setReference(mleftFrontEncoder.getPosition() , CANSparkMax.ControlType.kPosition);
      mrightPIDController.setReference(mrightFrontEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("set reference left", mleftFrontEncoder.getPosition());
      SmartDashboard.putNumber("set reference right", mrightFrontEncoder.getPosition());

    }
  }

  public void updatePIDController(SparkMaxPIDController pidController, double p, double i, double d, double minOutput, double maxOutput){
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setIZone(0);
    pidController.setFF(0);
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
    setGearMode(GearMode.HIGH_GEAR);
  }

  public void shiftDown(){
    mShifter.set(Value.kForward);
    gearRatio = Constants.GEAR_RATIO_LOW;
    setGearMode(GearMode.LOW_GEAR);
  }

  public double rotationsToMeters(double rotations){
    return rotations / gearRatio * 0.145 * Math.PI;
  }

  public double metersToRotations(double meters){
    return meters * gearRatio / 0.145 / Math.PI;
  }

  public DriveControlMode getDriveControlMode(){
    return mDriveControlMode;
  }

  public void setDriveMode(DriveControlMode mode){
    mDriveControlMode = mode;
  }

  public void setGearMode(GearMode mode){
    mGearMode = mode;
  }

  public void switchDriveMode(){
    if (mDriveControlMode == DriveControlMode.POSITION_CONTROL || mDriveControlMode == DriveControlMode.VELOCITY_CONTROL){
      setDriveMode(DriveControlMode.OPEN_LOOP);
      System.out.println("[Drive] Set to Open Loop");
    }
    else if (mDriveControlMode == DriveControlMode.OPEN_LOOP){
      setDriveMode(DriveControlMode.VELOCITY_CONTROL);
      configureVelocityControl();
      System.out.println("[Drive] Set to Velocity Control");
    }
  }

  public boolean isAtSetpoint(double setpoint){
    return rotationsToMeters(mleftFrontEncoder.getPosition()) == setpoint;
  }

  public double getHeading(){
    double headingAngle = mGyro.getAngle();
    while (headingAngle < 0) {
      headingAngle += 360;
    }
    while (headingAngle > 360) {
      headingAngle -= 360;
    }
    return headingAngle;
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
  
  public void turnToHeadingAngle(double headingAngle) {
    double turnAngle = headingAngle - mGyro.getAngle() % 360;

    if (turnAngle > 180.0) {
      turnAngle = - 360.0 + turnAngle;
    }

    setRotation(turnAngle);
  }

  public void turnToTarget() {
    double x = mPose.getX() - 4.11;
    double y = mPose.getY() - 2.055;
    double theta = mGyro.getAngle() % 360.0; // self angle
    double turnAngle;

    if (y > 0) {
      turnAngle = Math.atan(x / y) / Math.PI * 180.0 + 90.0 - theta;
    } else {
      turnAngle = Math.atan(x / y) / Math.PI * 180.0 - theta - 90.0;
    }
    
    if (turnAngle > 180.0) {
      turnAngle = - 360.0 + turnAngle;
    }

    setRotation(turnAngle);
  }

  public enum DriveControlMode{
    OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL
  } 

  public enum GearMode{
    HIGH_GEAR, LOW_GEAR
  }

}
