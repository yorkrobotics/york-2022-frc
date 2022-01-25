// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VelocityController extends SubsystemBase {
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;
  private SparkMaxPIDController left_pidController;
  private SparkMaxPIDController right_pidController;
  private RelativeEncoder left_encoder;
  private RelativeEncoder right_encoder;
  public double kP, kI, kD, kMaxOutput, kMinOutput, maxRPM;


  /** Creates a new VelocityController. */
  public VelocityController() {
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;
    maxRPM = 3000;

    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    leftBack = new CANSparkMax(2, MotorType.kBrushless);
    rightFront = new CANSparkMax(4, MotorType.kBrushless);
    rightBack = new CANSparkMax(3, MotorType.kBrushless);

    // leftFront.setInverted(false);
    // rightFront.setInverted(false);
    
    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    left_encoder = leftFront.getEncoder();
    right_encoder = rightFront.getEncoder();

    left_pidController = leftFront.getPIDController();
    right_pidController = rightFront.getPIDController();
    
    left_pidController.setP(kP);
    left_pidController.setI(kI);
    left_pidController.setD(kD);
    left_pidController.setIZone(0);
    left_pidController.setFF(1.5e-5);
    left_pidController.setOutputRange(kMinOutput, kMaxOutput);

    right_pidController.setP(kP);
    right_pidController.setI(kI);
    right_pidController.setD(kD);
    right_pidController.setIZone(0);
    right_pidController.setFF(1.5e-5);
    right_pidController.setOutputRange(kMinOutput, kMaxOutput);

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

  public void driveWithVelocityControl(XboxController controller){
    // get coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { left_pidController.setP(p); right_pidController.setP(p); kP = p; }
    if((i != kI)) { left_pidController.setI(i); right_pidController.setP(i); kI = i; }
    if((d != kD)) { left_pidController.setD(d); right_pidController.setP(d); kD = d; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      left_pidController.setOutputRange(min, max); 
      right_pidController.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }

    //controlling the speeds
    double velocity_value = (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())*maxRPM;
    double left_setPoint = velocity_value;
    double right_setPoint = velocity_value;

    left_pidController.setReference(right_setPoint, CANSparkMax.ControlType.kVelocity);
    right_pidController.setReference(left_setPoint, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("Left_SetPoint", left_setPoint);
    SmartDashboard.putNumber("Right_SetPoint", right_setPoint);

    SmartDashboard.putNumber("Left_velocity", left_encoder.getVelocity());
    SmartDashboard.putNumber("Right_velocity", right_encoder.getVelocity());

  }



}
