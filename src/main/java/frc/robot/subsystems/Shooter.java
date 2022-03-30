// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.ThroughBoreEncoder;

public class Shooter extends SubsystemBase {
  private static final Shooter mShooter = new Shooter();
  public static Shooter getInstance(){
    return mShooter;
  }

  private VictorSPX mTop, mBottom;

  private ThroughBoreEncoder mEncoderTop, mEncoderBottom;
  
  private double testSpeed;
  private double x_field;
  private double power;
  private boolean isShooting = false;


  /** Creates a new Shooter. */
  public Shooter() {
    mTop = new VictorSPX(Constants.VictorSPX.SHOOTER_TOP);
    mBottom = new VictorSPX(Constants.VictorSPX.SHOOTER_BOTTOM);

    mEncoderTop = new ThroughBoreEncoder(Constants.DIO.SHOOTER_ENCODER_TOP);
    mEncoderBottom = new ThroughBoreEncoder(Constants.DIO.SHOOTER_ENCODER_BOTTOM);

    mTop.setInverted(false);
    mBottom.setInverted(false);

    testSpeed = 0;

    SmartDashboard.putNumber("Test speed", testSpeed);
  }

  @Override
  public void periodic() {
    power = SmartDashboard.getNumber("Target Power", 0);
    SmartDashboard.putBoolean("Shooting", isShooting);
    // This method will be called once per scheduler run

    updateEncoderVelocities();

    SmartDashboard.putNumber("Encoder Top Velocity (raw)", mEncoderTop.getVelocityRaw());
    SmartDashboard.putNumber("Encoder Top Velocity (filtered)", mEncoderTop.getVelocityFiltered());
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
   * Get the average velocity of the shooter encoders.
   * @return (Top velocity + bottom velocity) / 2
   */
  public double getSpeed() {
    return (mEncoderTop.getVelocityFiltered() + mEncoderBottom.getVelocityFiltered()) / 2;
  }

  // shoots target with field x and characterized angle
  public void shootTarget() { 
    this.runShooter(power);
  }

  public boolean isShooting() {
    return isShooting;
  }

  private void updateEncoderVelocities() {
    mEncoderTop.updateVelocity();
    mEncoderBottom.updateVelocity();
  }
}
