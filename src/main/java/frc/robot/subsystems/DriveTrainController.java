// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainController extends SubsystemBase {
  private CANSparkMax leftFront, leftBack, rightFront, rightBack;

  public double kP, kI, kD, kMaxOutput, kMinOutput, maxRPM;

  private DifferentialDrive m_drive;

  private VelocityControlCANSparkMax v_leftFront, v_rightFront, v_leftBack, v_rightBack;


  /** Creates a new VelocityController. */
  public DriveTrainController() {
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;

    leftFront = new CANSparkMax(1, MotorType.kBrushless);
    leftBack = new CANSparkMax(2, MotorType.kBrushless);
    rightFront = new CANSparkMax(4, MotorType.kBrushless);
    rightBack = new CANSparkMax(3, MotorType.kBrushless);

    v_leftFront = new VelocityControlCANSparkMax(leftFront);
    v_rightFront = new VelocityControlCANSparkMax(rightFront);
    v_leftBack = new VelocityControlCANSparkMax(leftBack);
    v_rightBack = new VelocityControlCANSparkMax(rightBack);

    v_leftBack.follow(v_leftFront);
    v_rightBack.follow(v_rightFront);

    v_rightFront.setInverted(true);
    v_leftFront.setInverted(false);

    m_drive = new DifferentialDrive(v_leftFront, v_rightFront);
    
    //Set PID coefficients
    v_leftFront.setP(kP);
    v_leftFront.setI(kI);
    v_leftFront.setD(kD);
    v_leftFront.setIZone(0);
    v_leftFront.setFF(1.5e-5);
    v_leftFront.setOutputRange(kMinOutput, kMaxOutput);

    v_rightFront.setP(kP);
    v_rightFront.setI(kI);
    v_rightFront.setD(kD);
    v_rightFront.setIZone(0);
    v_rightFront.setFF(1.5e-5);
    v_rightFront.setOutputRange(kMinOutput, kMaxOutput);

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
    if((p != kP)) { v_leftFront.setP(p); v_rightFront.setP(p); kP = p; }
    if((i != kI)) { v_leftFront.setI(i); v_rightFront.setP(i); kI = i; }
    if((d != kD)) { v_leftFront.setD(d); v_rightFront.setP(d); kD = d; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      v_leftFront.setOutputRange(min, max); 
      v_rightFront.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }

    m_drive.arcadeDrive((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()), controller.getLeftX());

  }

  public void driveWithPositionControl(XboxController controller, double setMeters){
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    double setRotations = setMeters * Constants.GEAR_RATIO / 0.145 / Math.PI;
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { v_leftFront.setP(p); v_rightFront.setP(p); kP = p; }
    if((i != kI)) { v_leftFront.setI(i); v_rightFront.setP(i); kI = i; }
    if((d != kD)) { v_leftFront.setD(d); v_rightFront.setP(d); kD = d; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      v_leftFront.setOutputRange(min, max); 
      v_rightFront.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max; 
    }

    v_leftFront.setReference(setRotations, CANSparkMax.ControlType.kPosition);
    v_rightFront.setReference(setRotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("SetRotations", setRotations);
    SmartDashboard.putNumber("ProcessVariable", v_leftFront.getEncoder().getPosition());

  }

  public double getPosition(){
    return v_leftFront.get();
  }

}
