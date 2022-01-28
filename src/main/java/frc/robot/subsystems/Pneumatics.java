// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  private DoubleSolenoid m_doubleSolenoidPrimary, m_doubleSolenoidSecondary;

  /** Creates a new Pneumatics. */
  public Pneumatics() {
    m_doubleSolenoidPrimary = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 7);
    m_doubleSolenoidSecondary = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void gearShift(){
    
  }
}
