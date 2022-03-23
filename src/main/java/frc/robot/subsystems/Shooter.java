// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private static final Shooter mShooter = new Shooter();
  public static Shooter getInstance(){
    return mShooter;
  }

  private VictorSPX mTop, mBottom;
  private double testSpeed;

  private boolean isShooting = false;
  private double x_field;

  /** Creates a new Shooter. */
  public Shooter() {
    mTop = new VictorSPX(Constants.VictorSPX.SHOOTER_TOP);
    mBottom = new VictorSPX(Constants.VictorSPX.SHOOTER_BOTTOM);

    mTop.setInverted(false);
    mBottom.setInverted(false);

    testSpeed = 0;

    SmartDashboard.putNumber("Test speed", testSpeed);
  }

  @Override
  public void periodic() {
    x_field = SmartDashboard.getNumber("Field X", 0);
    // This method will be called once per scheduler run
  }

  /**
   * Run the top and bottom rollers of shooter
   * @param speed speed between (-1, 1)
   */
  public void runShooter(double speed) {
    mTop.set(VictorSPXControlMode.PercentOutput, speed);
    mBottom.set(VictorSPXControlMode.PercentOutput, speed);
    isShooting = speed > 0.0;
  }

  /**
   * Stop shooter motors
   */
  public void stopShooter() {
    mTop.set(VictorSPXControlMode.PercentOutput, 0);
    mBottom.set(VictorSPXControlMode.PercentOutput, 0);
    isShooting = false;
  }


  /**
   * TODO: Add comments
   * @param z
   * @param y
   * @param v
   * @return
   */
  public double calculate_trajectory_pos(double z, double y, double v) {
    double v_squared = Math.pow(v, 2);
    double v_fourth = Math.pow(v, 4);
    double z_squared = Math.pow(z, 2);
    double g = 9.8;
    double var_square_root = v_fourth - g * (g * z_squared + 2 * y * v_squared);
    double equation = (v_squared + Math.sqrt(var_square_root)) / (g * z);
    return Math.atan(equation);
  }
  
  /**
   * TODO: To be implemented
   * @return
   */
  public double getSpeed() {
    return 0;
  }

  // shoots target with field x and characterized angle
  public void shootTarget() {
    power = 0.513798 + 0.000825723 * x_field - 0.00000323753 * Math.pow(x_field, 2) + 8.5563 * Math.pow(10, -9) * Math.pow(x_field, 3);
    this.runShooter(power);
  }

  public boolean isShooting() {
    return isShooting;
  }
}
