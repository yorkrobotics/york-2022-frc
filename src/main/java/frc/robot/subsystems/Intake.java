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
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Intake extends SubsystemBase {
  private static final Intake mIntake = new Intake();
  public static Intake getInstance(){
    return mIntake;
  }

  private VictorSPX mRoller;
  private DoubleSolenoid mLifter;

  private IntakeLifterMode mLifterMode;

  public Intake() {
    mRoller = new VictorSPX(Constants.VictorSPX.INTAKE_ROLLER); //channel number 2
    mRoller.setInverted(false);

    mLifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PCM.INTAKE_FORWARD, Constants.PCM.INTAKE_REVERSE);
    mLifterMode = IntakeLifterMode.UNKNOWN;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runRoller(double speed) {
    mRoller.set(VictorSPXControlMode.PercentOutput, speed);
  }

  public void stopRoller() {
    mRoller.set(VictorSPXControlMode.Disabled, 0);
  }

  public void deploy(){
    mLifter.set(Value.kForward);
    mLifterMode = IntakeLifterMode.DEPLOYED;
  }

  public void retract(){
    stopRoller();
    mLifter.set(Value.kReverse);
    mLifterMode = IntakeLifterMode.RETRACTED;
  }

  public void release(){
    stopRoller();
    mLifter.set(Value.kOff);
  }

  public enum IntakeLifterMode{
    RETRACTED, DEPLOYED, UNKNOWN
  }
}
