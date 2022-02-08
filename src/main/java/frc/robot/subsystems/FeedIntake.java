// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class FeedIntake extends SubsystemBase {
  double speed;
  PWMMotorController intake;

  /** Creates a new ExampleSubsystem. */
  public FeedIntake() {
    speed = 0;
    intake = new PWMVictorSPX(2);
    intake.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double spd) {
    speed = spd;
    intake.set(spd);
  }

  public double getSpeed() {
    return speed;
  }

  public void stopMotor() {
    setSpeed(0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
