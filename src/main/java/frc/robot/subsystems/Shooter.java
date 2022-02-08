// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private PWMVictorSPX top, bottom;
  double speed;

  /** Creates a new Shooter. */
  public Shooter() {
    top = new PWMVictorSPX(0);
    bottom = new PWMVictorSPX(1);

    top.setInverted(false);
    bottom.setInverted(false);
  }

  public void setSpeed(double spd) {
    speed = spd;
    top.set(spd);
    bottom.set(spd);
  }

  public void stopMotor() {
    top.set(0);
    bottom.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
