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

  public double getSpeed() {
    return speed;
  }

  
  public void stopMotor() {
    top.set(0);
    bottom.set(0);
  }
  
  public double calculate_trajectory_pos(double z, double y, double v) {
    double v_squared = Math.pow(v, 2);
    double v_fourth = Math.pow(v, 4);
    double z_squared = Math.pow(z, 2);
    double g = 9.8;
    double var_square_root = v_fourth - g * (g * z_squared + 2 * y * v_squared);
    double equation = (v_squared + Math.sqrt(var_square_root)) / (g * z);
    return Math.atan(equation);
  }
  
  public void setAngle(double angle) {
    // sets the angle of the shooter
  }

  // public double calculate_trajectory_neg(double x, double y, double v) {
  //   double v_squared = Math.pow(v, 2);
  //   double v_fourth = Math.pow(v, 4);
  //   double x_squared = Math.pow(x, 2);
  //   double g = 9.8;
  //   double var_square_root = v_fourth - g * (g * x_squared + 2 * y * v_squared);
  //   double equation = (v_squared - Math.sqrt(var_square_root)) / (g * x);
  //   return Math.atan(equation);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
