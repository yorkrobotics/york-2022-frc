// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private NetworkTableEntry kPEntry, kIEntry, kDEntry, kMinOutputEntry, kMaxOutputEntry;

  private DifferentialDriveKinematics mKinematics;
  private DifferentialDriveOdometry mOdometry;
  private ADXRS450_Gyro mGyro;
  private Pose2d mPose;
  private final Field2d mField = new Field2d();
  private SimpleMotorFeedforward mFeedforward;
  private double totalDistanceLeft = 0, totalDistanceRight = 0;
  private double lastLeftEncoderPos = 0, lastRightEncoderPos = 0;

  private ShuffleboardTab driveTab;

  private PIDController mLeftAutoController = new PIDController(Constants.kP_AUTO, 0, 0), 
  mRightAutoController = new PIDController(Constants.kP_AUTO, 0, 0);

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

    setToOpenLoopMode(); //Default drive mode set to open loop
    mGearMode = GearMode.UNKNOWN;
    shiftToLowGear(); // Always set to low gear at the start

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
    mPose = new Pose2d();

    // Update to Shuffleboard
    driveTab = Shuffleboard.getTab("Drive");

    driveTab.addNumber("Pose X", () -> this.getPose().getX());
    driveTab.addNumber("Pose Y", () -> this.getPose().getY());
    driveTab.addNumber("Left Wheel Speeds", () -> this.getWheelSpeeds().leftMetersPerSecond);
    driveTab.addNumber("Right Wheel Speeds", () -> this.getWheelSpeeds().rightMetersPerSecond);
    driveTab.addNumber("Left Encoder Meters", ()-> this.rotationsToMeters(mleftFrontEncoder.getPosition()));
    driveTab.addNumber("right Encoder Meters", ()-> this.rotationsToMeters(mrightFrontEncoder.getPosition()));
    driveTab.add(mGyro);

    kPEntry = driveTab.add("P Gain", kP).getEntry();
    kIEntry = driveTab.add("I Gain", kI).getEntry();
    kDEntry = driveTab.add("D Gain", kD).getEntry();
    kMinOutputEntry = driveTab.add("Min Output", kMinOutput).getEntry();
    kMaxOutputEntry = driveTab.add("Max Output", kMaxOutput).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mGearMode == GearMode.UNKNOWN){
      lastLeftEncoderPos = mleftFrontEncoder.getPosition();
      lastRightEncoderPos = mrightFrontEncoder.getPosition();
    }
    else{
      updateOdometry();
    }
    
    mPose = mOdometry.update(mGyro.getRotation2d(), totalDistanceLeft, totalDistanceRight);
    mField.setRobotPose(mOdometry.getPoseMeters());
  
  }

  /**
   * Run the motors openloop 
   * @param left_velocity left velocity between -1 to 1
   * @param right_velocity right velocity between -1 to 1
   */
  public void setOpenLoop(double left_velocity, double right_velocity){
    if (mDriveControlMode == DriveControlMode.OPEN_LOOP){
      mleftFront.set(left_velocity * Constants.MAX_OPENLOOP_SPEED);
      mrightFront.set(right_velocity * Constants.MAX_OPENLOOP_SPEED);
    }
    else{
      System.out.println("[Drive] drive mode not in open loop");
    }
  }

  /**
   * Run the motors to a velocity setpoint between -1 to 1
   * @param left_velocity left velocity 
   * @param right_velocity right velocity
   */
  public void setVelocity(double left_velocity, double right_velocity){
    if (mDriveControlMode == DriveControlMode.VELOCITY_CONTROL){
      mleftPIDController.setReference(left_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
      mrightPIDController.setReference(right_velocity * Constants.DRIVE_MAX_RPM, CANSparkMax.ControlType.kVelocity);
    }
    else{
      System.out.println("[Drive] drive mode not in velocity control");
    }
  }

  /**
   * Run the motors to a position setpoint
   * @param setMeters setpoint in meters
   */
  public void setPosition(double setMeters){
    double setRotations = metersToRotations(setMeters);
    mleftPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
    mrightPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Run the motors to an angle setpoint
   * @param setAngle setpoint in degrees
   */
  public void setRotation(double setAngle){
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

  /**
   * Set voltages to motors
   * @param leftVolts left motor voltage
   * @param rightVolts right motor voltage
   */
  public void tankDriveVolts(double leftVolts, double rightVolts){
    mleftFront.setVoltage(leftVolts);
    mrightFront.setVoltage(rightVolts);
  }

  /**
   * Switches drive mode between open loop and velocity control
   */
  public void switchDriveMode(){
    if (mDriveControlMode == DriveControlMode.POSITION_CONTROL || mDriveControlMode == DriveControlMode.VELOCITY_CONTROL){
      setToOpenLoopMode();;
      System.out.println("[Drive] Set to Open Loop");
    }
    else if (mDriveControlMode == DriveControlMode.OPEN_LOOP){
      setToVelocityMode();
      configureVelocityControl();
      System.out.println("[Drive] Set to Velocity Control");
    }
  }

  /**
   * Arcade drive that uses controller triggers and joystick
   * @param controller xbox controller
   * @return wheelspeeds
   */
  public WheelSpeeds mArcadeDrive(XboxController controller){
    // double controller_leftX = controller.getLeftX();
    // if (Math.abs(controller_leftX) < 0.25) controller_leftX = 0;
    return DifferentialDrive.arcadeDriveIK(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis(), controller.getLeftX(), true);
    
  }

  /**
   * Shift to high gear
   */
  public void shiftToHighGear(){
    mShifter.set(Value.kReverse);
    gearRatio = Constants.GEAR_RATIO_HIGH;
    mGearMode = GearMode.HIGH_GEAR;
  }

  /**
   * Shift to low gear
   */
  public void shiftToLowGear(){
    mShifter.set(Value.kForward);
    gearRatio = Constants.GEAR_RATIO_LOW;
    mGearMode = GearMode.LOW_GEAR;
  }

  /**
   * Convert motor rotations to meters
   * @param rotations rotation
   * @return meter
   */
  public double rotationsToMeters(double rotations){
    return rotations / gearRatio * 0.145 * Math.PI;
  }

  /**
   * Convert meters to motor rotations
   * @param meters meter
   * @return rotation
   */
  public double metersToRotations(double meters){
    return meters * gearRatio / 0.145 / Math.PI;
  }

  /**
   * Get the drive mode
   * @return drive mode
   */
  public DriveControlMode getDriveControlMode(){
    return mDriveControlMode;
  }

  /**
   * Set drive mode to open loop
   */
  public void setToOpenLoopMode(){
    mDriveControlMode = DriveControlMode.OPEN_LOOP;
  }

  /**
   * Set drive mode to velocity control
   */
  public void setToVelocityMode(){
    configureVelocityControl();
    mDriveControlMode = DriveControlMode.VELOCITY_CONTROL;
  }

  /**
   * Set drive mode to position control
   */
  public void setToPositionMode(){
    configurePositionControl();
    mDriveControlMode = DriveControlMode.POSITION_CONTROL;
  }

  /**
   * Set PID gains and min, max output of a PID controller
   * @param pidController SparkMax PID controller
   * @param p P gain
   * @param i I gain
   * @param d D gain
   * @param minOutput minimum output
   * @param maxOutput maximum output
   */
  public void setPIDController(SparkMaxPIDController pidController, double p, double i, double d, double minOutput, double maxOutput){
    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setOutputRange(minOutput, maxOutput);
  }

  /**
   * Get the gyro angle
   * @return gyro angle between 0-360
   */
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

  /**
   * Get the motor feedforward
   * @return motor feedforward
   */
  public SimpleMotorFeedforward getFeedForward(){
    return mFeedforward;
  }

  /**
   * Get the kinematics
   * @return kinematics
   */
  public DifferentialDriveKinematics getKinematics(){
    return mKinematics;
  }

  /**
   * Get the odometry
   * @return odometry
   */
  public DifferentialDriveOdometry getOdometry(){
    return mOdometry;
  }

  /**
   * Get the current pose
   * @return current pose
   */
  public Pose2d getPose(){
    return mPose;
  }

  /**
   * Get the wheelspeeds in m/s
   * @return the wheelspeeds in m/s
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      rotationsToMeters(mleftFrontEncoder.getVelocity()) / 60, 
      rotationsToMeters(mrightFrontEncoder.getVelocity()) / 60);
  }

  /**
   * Reset the encoders, the stored last encoder positions and total distance travelled
   */
  public void resetEncoders(){
    mleftFrontEncoder.setPosition(0);
    mrightFrontEncoder.setPosition(0);

    lastLeftEncoderPos = 0;
    lastRightEncoderPos = 0;
    totalDistanceLeft = 0;
    totalDistanceRight = 0;
  }

  /**
   * Reset the odometry to a pose and reset the enconders
   * @param pose The pose to reset to
   */
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    mOdometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));
  }

  /**
   * Get gyro angle
   * @return gyro angle
   */
  public double getGyroAngle(){
    return mGyro.getAngle();
  }
  
  /**
   * Turn the robot the other direction if the angle is larger than 180 degrees
   * @param headingAngle The angle to turn between 0-360 degrees
   */
  public void turnToHeadingAngle(double headingAngle) {
    double turnAngle = headingAngle - mGyro.getAngle() % 360;

    if (turnAngle > 180.0) {
      turnAngle = - 360.0 + turnAngle;
    }
    setRotation(turnAngle);
  }

  /**
   * Turn to the center hub of the field
   */
  public void turnToTarget() {
    double x = mPose.getX() - Constants.FIELD_CENTER_X;
    double y = mPose.getY() - Constants.FIELD_CENTER_Y;
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
    setToOpenLoopMode();
  }

  /**
   * Takes in a trajectory and returns a ramsete command
   * @param trajectory An Autonomous trajectory
   * @param resetOdometry reset the odometry
   * @return a ramsete command
   */
  public Command makeTrajectoryToCommand(Trajectory trajectory, Boolean resetOdometry){
    RamseteCommand command =  new RamseteCommand(
        trajectory, 
        this::getPose, 
        new RamseteController(2.0, 0.7), 
        this.getFeedForward(), 
        this.getKinematics(), 
        this::getWheelSpeeds, 
        mLeftAutoController, 
        mRightAutoController, 
        this::tankDriveVolts, 
        this);
    return resetOdometry ? 
        new SequentialCommandGroup(
            new InstantCommand(()-> this.resetOdometry(trajectory.getInitialPose()), this), 
            command.andThen(()->this.tankDriveVolts(0, 0)))
            : command.andThen(()->this.tankDriveVolts(0, 0));
  }

  /**
   * Update the odometry according to 
   * 1. the current gyro angle and 
   * 2. the distance traveled by each encoder since the last update
   */
  public void updateOdometry(){
    double diffLeft = mleftFrontEncoder.getPosition() - lastLeftEncoderPos;
    double diffRight = mrightFrontEncoder.getPosition() - lastRightEncoderPos;
    
    double leftMeters = rotationsToMeters(diffLeft);
    double rightMeters = rotationsToMeters(diffRight);
    
    totalDistanceLeft += leftMeters;
    totalDistanceRight += rightMeters;

    lastLeftEncoderPos = mleftFrontEncoder.getPosition();
    lastRightEncoderPos = mrightFrontEncoder.getPosition();
  }

  /**
   * Set PID gains for velocity control
   */
  private void configureVelocityControl(){
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
    // if(kPEntry.getDouble(0) != kP) kP = kPEntry.getDouble(0);
    // if(kIEntry.getDouble(0) != kI) kI = kIEntry.getDouble(0);
    // if(kDEntry.getDouble(0) != kD) kD = kDEntry.getDouble(0);
    // if(kMaxOutputEntry.getDouble(0) != kMaxOutput) kMaxOutput = kMaxOutputEntry.getDouble(0);
    // if(kMinOutputEntry.getDouble(0) != kMinOutput) kMinOutput = kMinOutputEntry.getDouble(0);

    setPIDController(mleftPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    setPIDController(mrightPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
  }

  /**
   * Set PID gains for position control
   */
  private void configurePositionControl(){
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
    // if(kPEntry.getDouble(0) != kP) kP = kPEntry.getDouble(0);
    // if(kIEntry.getDouble(0) != kI) kI = kIEntry.getDouble(0);
    // if(kDEntry.getDouble(0) != kD) kD = kDEntry.getDouble(0);
    // if(kMaxOutputEntry.getDouble(0) != kMaxOutput) kMaxOutput = kMaxOutputEntry.getDouble(0);
    // if(kMinOutputEntry.getDouble(0) != kMinOutput) kMinOutput = kMinOutputEntry.getDouble(0);

    setPIDController(mleftPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    setPIDController(mrightPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
  }

  public enum DriveControlMode{
    OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL
  } 

  public enum GearMode{
    HIGH_GEAR, LOW_GEAR, UNKNOWN
  }

}
