// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
  private static final DriveTrain mDrive = new DriveTrain();
  public static DriveTrain getInstance() {
    return mDrive;
  }

  //Hardware setup
  private CANSparkMax mLeftFront, mLeftBack, mRightFront, mRightBack;
  private RelativeEncoder mLeftFrontEncoder, mRightFrontEncoder;
  private DoubleSolenoid mShifter;

  //Controller setup
  private SparkMaxPIDController mleftPIDController, mrightPIDController;

  private DriveControlMode mDriveControlMode;
  private GearMode mGearMode;
  private boolean isInvertedDriving;

  private double gearRatio;
  private double kP, kI, kD, kMinOutput, kMaxOutput;
  private double totalDistanceLeft = 0, totalDistanceRight = 0;
  private double lastLeftEncoderPos = 0, lastRightEncoderPos = 0;

  private DifferentialDriveKinematics mKinematics;
  private DifferentialDriveOdometry mOdometry;
  private ADXRS450_Gyro mGyro;

  private SlewRateLimiter mTeleopRateLimiter = new SlewRateLimiter(Constants.FORWARDS_SLEW_RATE_LIMIT);

  private ShuffleboardTab driveTab;
  private final Field2d mField;


  public DriveTrain() {
    //Hardware
    mLeftFront = new CANSparkMax(Constants.SparkMax.DRIVE_LEFT_FRONT, MotorType.kBrushless);
    mLeftBack = new CANSparkMax(Constants.SparkMax.DRIVE_LEFT_BACK, MotorType.kBrushless);
    mRightFront = new CANSparkMax(Constants.SparkMax.DRIVE_RIGHT_FRONT, MotorType.kBrushless);
    mRightBack = new CANSparkMax(Constants.SparkMax.DRIVE_RIGHT_BACK, MotorType.kBrushless);

    mShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PCM.DRIVE_GEAR_SHIFT_FORWARD, Constants.PCM.DRIVE_GEAR_SHIFT_REVERSE);


    mLeftFront.setIdleMode(Constants.DRIVE_IDLE_MODE);
    mLeftBack.setIdleMode(Constants.DRIVE_IDLE_MODE);
    mRightFront.setIdleMode(Constants.DRIVE_IDLE_MODE);
    mRightBack.setIdleMode(Constants.DRIVE_IDLE_MODE);

    mLeftBack.follow(mLeftFront);
    mRightBack.follow(mRightFront);

    mLeftFront.setInverted(false);
    mRightFront.setInverted(true);

    mLeftFrontEncoder = mLeftFront.getEncoder();
    mRightFrontEncoder = mRightFront.getEncoder();
    
    //Controller
    mleftPIDController = mLeftFront.getPIDController();
    mrightPIDController = mRightFront.getPIDController();

    mLeftFront.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);
    mRightFront.getReverseLimitSwitch(Type.kNormallyOpen).enableLimitSwitch(false);

    setToOpenLoopMode(); //Default drive mode set to open loop
    mGearMode = GearMode.UNKNOWN;
    shiftToHighGear();; // Always set to low gear at the start
    isInvertedDriving = false;

    mGyro = new ADXRS450_Gyro();
    mGyro.calibrate();

    mField = new Field2d();
    Shuffleboard.getTab("Autonomous").add(mField);

    mKinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
    mOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(this.getGyroAngle()));

    // Update to Shuffleboard
    driveTab = Shuffleboard.getTab("Drive");

    driveTab.addNumber("Pose X", () -> this.getPose().getX());
    driveTab.addNumber("Pose Y", () -> this.getPose().getY());
    driveTab.addNumber("Left Wheel Speeds", () -> this.getWheelSpeeds().leftMetersPerSecond);
    driveTab.addNumber("Right Wheel Speeds", () -> this.getWheelSpeeds().rightMetersPerSecond);
    driveTab.addNumber("Left Encoder Meters", ()-> this.rotationsToMeters(mLeftFrontEncoder.getPosition()));
    driveTab.addNumber("right Encoder Meters", ()-> this.rotationsToMeters(mRightFrontEncoder.getPosition()));
    driveTab.add(mGyro);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (mGearMode == GearMode.UNKNOWN){
      lastLeftEncoderPos = mLeftFrontEncoder.getPosition();
      lastRightEncoderPos = mRightFrontEncoder.getPosition();
    }
    else{
      updateOdometry();
    }
    
    mOdometry.update(mGyro.getRotation2d(), totalDistanceLeft, totalDistanceRight);
    mField.setRobotPose(mOdometry.getPoseMeters());
  
    SmartDashboard.putBoolean("driving is inverted", isInvertedDriving);
  }

  /**
   * Run the motors openloop 
   * @param left_velocity left velocity between -1 to 1
   * @param right_velocity right velocity between -1 to 1
   */
  public void driveOpenloop(double left_velocity, double right_velocity){
    if (mDriveControlMode == DriveControlMode.OPEN_LOOP){
      mLeftFront.set(left_velocity * Constants.MAX_OPENLOOP_SPEED);
      mRightFront.set(right_velocity * Constants.MAX_OPENLOOP_SPEED);
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
  public void driveVelocitySetpoint(double left_velocity, double right_velocity){
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
  public void drivePositionSetpoint(double setMeters){
    double setRotations = metersToRotations(setMeters);
    mleftPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
    mrightPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Run the motors to an angle setpoint
   * @param setAngle setpoint in degrees
   */
  public void driveRotationSetpoint(double setAngle){
    mleftPIDController.setOutputRange(-0.5, 0.5);
    mleftPIDController.setP(.3);
    mrightPIDController.setOutputRange(-0.5, 0.5);
    mrightPIDController.setP(.3);

    mleftPIDController.setD(10);
    mrightPIDController.setD(10);

    SmartDashboard.putNumber("set angle", setAngle);
    if (setAngle != 0 ) {
      double setMeters = setAngle / 360 * Math.PI * Constants.TRACK_WIDTH;
      mleftPIDController.setReference(mLeftFrontEncoder.getPosition() + metersToRotations(setMeters), CANSparkMax.ControlType.kPosition);
      mrightPIDController.setReference(mRightFrontEncoder.getPosition() - metersToRotations(setMeters), CANSparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("set reference left", mLeftFrontEncoder.getPosition() + metersToRotations(setMeters));
      SmartDashboard.putNumber("set reference right", mRightFrontEncoder.getPosition() - metersToRotations(setMeters));
    } else {
      mleftPIDController.setReference(mLeftFrontEncoder.getPosition() , CANSparkMax.ControlType.kPosition);
      mrightPIDController.setReference(mRightFrontEncoder.getPosition(), CANSparkMax.ControlType.kPosition);
      SmartDashboard.putNumber("set reference left", mLeftFrontEncoder.getPosition());
      SmartDashboard.putNumber("set reference right", mRightFrontEncoder.getPosition());
    }
  }

  /**
   * Set voltages to motors
   * @param leftVolts left motor voltage
   * @param rightVolts right motor voltage
   */
  public void tankDriveVolts(double leftVolts, double rightVolts){
    mLeftFront.setVoltage(leftVolts);
    mRightFront.setVoltage(rightVolts);
  }

  /**
   * Stop the motors 
   */
  public void stopDriveMotors(){
    mLeftFront.stopMotor();
    mLeftBack.stopMotor();
    mRightFront.stopMotor();
    mRightBack.stopMotor();
  }

  /**
   * Switches drive mode between open loop and velocity control
   */
  public void switchDriveMode(){
    if (mDriveControlMode == DriveControlMode.POSITION_CONTROL || mDriveControlMode == DriveControlMode.VELOCITY_CONTROL){
      setToOpenLoopMode();;
      System.out.println("[Drive] Open Loop");
    }
    else if (mDriveControlMode == DriveControlMode.OPEN_LOOP){
      setToVelocityMode();
      configureVelocityControl();
      System.out.println("[Drive] Velocity Control");
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
    double forwards = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    double slewLimitedForwards = mTeleopRateLimiter.calculate(forwards);

    double trueForwards = slewLimitedForwards; // Change this to `forwards` to eliminate slew rate limiter

    if (isInvertedDriving) trueForwards *= -1;

    return DifferentialDrive.arcadeDriveIK(trueForwards, controller.getLeftX(), true);
    
  }

  public void switchInvertedDriving(){
    if (isInvertedDriving){
      isInvertedDriving = false;
    }
    else{
      isInvertedDriving = true;
    }
  }

  /**
   * Shift to high gear
   */
  public void shiftToHighGear(){
    mShifter.set(Value.kReverse);
    gearRatio = Constants.GEAR_RATIO_HIGH;
    mGearMode = GearMode.HIGH_GEAR;
    System.out.println("[Drive] High Gear");
  }

  /**
   * Shift to low gear
   */
  public void shiftToLowGear(){
    mShifter.set(Value.kForward);
    gearRatio = Constants.GEAR_RATIO_LOW;
    mGearMode = GearMode.LOW_GEAR;
    System.out.println("[Drive] Low Gear");
  }

  /**
   * Convert motor rotations to meters
   * @param rotations rotation
   * @return meter
   */
  public double rotationsToMeters(double rotations){
    return rotations / gearRatio * Constants.WHEEL_DIAMETER * Math.PI;
  }

  /**
   * Convert meters to motor rotations
   * @param meters meter
   * @return rotation
   */
  public double metersToRotations(double meters){
    return meters * gearRatio / Constants.WHEEL_DIAMETER / Math.PI;
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
  public void configurePIDController(SparkMaxPIDController pidController, double p, double i, double d, double minOutput, double maxOutput){
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
    switch (mGearMode){
      case LOW_GEAR:
        return new SimpleMotorFeedforward(Constants.kS_LOW_GEAR, Constants.kV_LOW_GEAR, Constants.kA_LOW_GEAR);
      case HIGH_GEAR:
        return new SimpleMotorFeedforward(Constants.kS_HIGH_GEAR, Constants.kV_HIGH_GEAR, Constants.kA_HIGH_GEAR);
      default:
        return null;
    }
  }

  public PIDController getAutoPIDController() {
    switch (mGearMode){
      case LOW_GEAR:
        return new PIDController(Constants.kP_AUTO_LOW_GEAR, 0, 0);
      case HIGH_GEAR:
        return new PIDController(Constants.kP_AUTO_HIGH_GEAR, 0, 0);
      default:
        return null;
    }

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
    return mOdometry.getPoseMeters();
  }

  /**
   * Get the wheelspeeds in m/s
   * @return the wheelspeeds in m/s
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(
      rotationsToMeters(mLeftFrontEncoder.getVelocity()) / 60, 
      rotationsToMeters(mRightFrontEncoder.getVelocity()) / 60);
  }

  /**
   * Reset the encoders, the stored last encoder positions and total distance travelled
   */
  public void resetEncoders(){
    mLeftFrontEncoder.setPosition(0);
    mRightFrontEncoder.setPosition(0);

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
    mOdometry.resetPosition(pose, mGyro.getRotation2d());
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
    driveRotationSetpoint(turnAngle);
  }

  /**
   * Turn to the center hub of the field
   */
  public void turnToTarget() {
    double x = this.getPose().getX() - Constants.FIELD_CENTER_X;
    double y = this.getPose().getY() - Constants.FIELD_CENTER_Y;
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

    driveRotationSetpoint(turnAngle);
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
        this.getAutoPIDController(), 
        this.getAutoPIDController(), 
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
    double diffLeft = mLeftFrontEncoder.getPosition() - lastLeftEncoderPos;
    double diffRight = mRightFrontEncoder.getPosition() - lastRightEncoderPos;
    
    double leftMeters = rotationsToMeters(diffLeft);
    double rightMeters = rotationsToMeters(diffRight);
    
    totalDistanceLeft += leftMeters;
    totalDistanceRight += rightMeters;

    lastLeftEncoderPos = mLeftFrontEncoder.getPosition();
    lastRightEncoderPos = mRightFrontEncoder.getPosition();
  }

  /**
   * Set PID gains for velocity control
   */
  private void configureVelocityControl(){
    if (mGearMode == GearMode.LOW_GEAR){
      kP = Constants.kP_VELOCITY_DRIVE_LOW_GEAR;
      kI = Constants.kI_VELOCITY_DRIVE_LOW_GEAR;
      kD = Constants.kD_VELOCITY_DRIVE_LOW_GEAR;
      kMinOutput = -0.7;
      kMaxOutput = 0.7;
    }
    if (mGearMode == GearMode.HIGH_GEAR){
      kP = Constants.kP_VELOCITY_DRIVE_HIGH_GEAR;
      kI = Constants.kI_VELOCITY_DRIVE_HIGH_GEAR;
      kD = Constants.kD_VELOCITY_DRIVE_HIGH_GEAR;
      kMinOutput = -0.7;
      kMaxOutput = 0.7;
    }

    configurePIDController(mleftPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    configurePIDController(mrightPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
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

    configurePIDController(mleftPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
    configurePIDController(mrightPIDController, kP, kI, kD, kMinOutput, kMaxOutput);
  }

  public boolean getInvertedDriving(){
    return isInvertedDriving;
  }

  public enum DriveControlMode{
    OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL
  } 

  public enum GearMode{
    HIGH_GEAR, LOW_GEAR, UNKNOWN
  }

}
