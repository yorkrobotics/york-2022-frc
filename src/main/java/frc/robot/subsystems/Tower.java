// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  /** Creates a new Tower. */
  public Tower() {
    mLeftMotor = new CANSparkMax(Constants.CAN.TOWER_ACTUATOR_LEFT, MotorType.kBrushless);
    mRightMotor = new CANSparkMax(Constants.CAN.TOWER_ACTUATOR_RIGHT, MotorType.kBrushless);

    mLeftController = mLeftMotor.getPIDController();
    mRightController = mRightMotor.getPIDController();

    mLeftEncoder = mLeftMotor.getEncoder();
    mRightEncoder = mRightMotor.getEncoder();

    mActuatorMode = TowerActuatorMode.OPEN_LOOP;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Tower left position", mLeftEncoder.getPosition());
    SmartDashboard.putNumber("Tower right position", mRightEncoder.getPosition());
  }

  public void runActuators(Double speed){
    mLeftMotor.set(speed * 0.2);
    mRightMotor.set(speed * 0.2);
  }

  private enum TowerActuatorMode{
    POSITION_CONTROL, OPEN_LOOP
  }
}
