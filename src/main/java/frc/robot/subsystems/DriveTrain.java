// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private CANSparkMax leftFront, leftBack, rightFront, rightBack;

  public double kP, kI, kD, kMaxOutput, kMinOutput, maxRPM, gearRatio;

  private SparkMaxPIDController leftPIDController, rightPIDController;

  /** Creates a new VelocityController. */
  public DriveTrain() {
    kP = 6e-6; 
    kI = 5e-7;
    kD = 1e-6; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;

    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    leftBack = new CANSparkMax(2, MotorType.kBrushless);
    rightFront = new CANSparkMax(4, MotorType.kBrushless);
    rightBack = new CANSparkMax(3, MotorType.kBrushless);

    leftFront.restoreFactoryDefaults();
    leftBack.restoreFactoryDefaults();
    rightFront.restoreFactoryDefaults();
    rightBack.restoreFactoryDefaults();

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftFront.setInverted(false);
    rightFront.setInverted(true);

    leftPIDController = leftFront.getPIDController();
    rightPIDController = rightFront.getPIDController();
    
    //Set PID coefficients
    leftPIDController.setP(kP);
    leftPIDController.setI(kI);
    leftPIDController.setD(kD);
    leftPIDController.setIZone(0);
    leftPIDController.setFF(1.5e-5);
    leftPIDController.setOutputRange(kMinOutput, kMaxOutput);

    rightPIDController.setP(kP);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);
    rightPIDController.setIZone(0);
    rightPIDController.setFF(1.5e-5);
    rightPIDController.setOutputRange(kMinOutput, kMaxOutput);

    // Display PID coefficients to SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }

  public void driveWithPositionControl(XboxController controller, double setMeters){
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    double setRotations = setMeters * gearRatio / 0.145 / Math.PI;
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { leftPIDController.setP(p); rightPIDController.setP(p); kP = p; }
    if((i != kI)) { leftPIDController.setI(i); rightPIDController.setI(i); kI = i; }
    if((d != kD)) { leftPIDController.setD(d); rightPIDController.setD(d); kD = d; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      leftPIDController.setOutputRange(min, max); 
      rightPIDController.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }

    leftFront.getEncoder().setPosition(0);
    rightFront.getEncoder().setPosition(0);

    leftPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);
    rightPIDController.setReference(setRotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("SetRotations", setRotations);
    SmartDashboard.putNumber("left_encoder", leftFront.getEncoder().getPosition());
    SmartDashboard.putNumber("right_encoder", rightFront.getEncoder().getPosition());

  }


  public void mArcadeDrive(XboxController controller){

    // get coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { leftPIDController.setP(p); rightPIDController.setP(p); kP = p; }
    if((i != kI)) { leftPIDController.setI(i); rightPIDController.setI(i); kI = i; }
    if((d != kD)) { leftPIDController.setD(d); rightPIDController.setD(d); kD = d; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      leftPIDController.setOutputRange(min, max); 
      rightPIDController.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }

    double xSpeed = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    double zRotaion = controller.getLeftX();

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

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (maxMagnitude > 1.0) {
      leftSpeed /= maxMagnitude;
      rightSpeed /= maxMagnitude;
    }

    SmartDashboard.putNumber("leftSpeed", leftSpeed);
    SmartDashboard.putNumber("rightSpeed", rightSpeed);

    leftPIDController.setReference(leftSpeed * Constants.MAX_RPM, CANSparkMax.ControlType.kVelocity);
    rightPIDController.setReference(rightSpeed * Constants.MAX_RPM, CANSparkMax.ControlType.kVelocity);

  }
  public void setToHighGear(){
    gearRatio = Constants.GEAR_RATIO_HIGH;
  }
  public void setToLowGear(){
    gearRatio = Constants.GEAR_RATIO_LOW;
  }
}
