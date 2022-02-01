// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private PWMVictorSPX top, bottom;

  /** Creates a new Shooter. */
  public Shooter() {
    top = new PWMVictorSPX(0);
    bottom = new PWMVictorSPX(1);

    top.setInverted(false);
    bottom.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(XboxController controller){
    top.set(controller.getRightY() * 0.5);
    bottom.set(controller.getLeftY() * 0.6);

    SmartDashboard.putNumber("Top", controller.getRightY());
    SmartDashboard.putNumber("Bottom", controller.getLeftY());
  }
}
