// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
  private static final Intake mIntake = new Intake();
  public static Intake getInstance(){
    return mIntake;
  }

  private VictorSPX mRoller;
  private DoubleSolenoid mLifter;

  private IntakeLifterMode mLifterMode;

  public Intake() {
    mRoller = new VictorSPX(Constants.VictorSPX.INTAKE_ROLLER);
    mRoller.setInverted(false);

    mLifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PCM.INTAKE_FORWARD, Constants.PCM.INTAKE_REVERSE);
    mLifterMode = IntakeLifterMode.UNKNOWN;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Run the roller
   * @param speed speed between (-1, 1)
   */
  public void runRoller(double speed) {
    mRoller.set(VictorSPXControlMode.PercentOutput, speed);
  }

  /**
   * Stop the motor of roller
   */
  public void stopRoller() {
    mRoller.set(VictorSPXControlMode.PercentOutput, 0);
  }

  /**
   * Deploy intake and set mode to deployed
   */
  public void deploy(){
    mLifter.set(Value.kForward);
    mLifterMode = IntakeLifterMode.DEPLOYED;
  }

  /**
   * Retract intake and set mode to retracted
   */
  public void retract(){
    stopRoller();
    mLifter.set(Value.kReverse);
    mLifterMode = IntakeLifterMode.RETRACTED;
  }

  /**
   * Release intake (set solenoid to off) and set mode to unknown
   */
  public void release(){
    stopRoller();
    mLifter.set(Value.kOff);
    mLifterMode = IntakeLifterMode.UNKNOWN;
  }

  /**
   * Get intake lifter mode
   * @return intake lifter mode
   */
  public IntakeLifterMode getIntakeLifterMode(){
    return mLifterMode;
  }

  public enum IntakeLifterMode{
    RETRACTED, DEPLOYED, UNKNOWN
  }
}
