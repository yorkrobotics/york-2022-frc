// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class PyCamera extends SubsystemBase {
  public Number[] hoop_coord;
  public double x;
  public double y;
  public double z;
  public NetworkTable table;
  
  /** Creates a new PyCamera. */
  public PyCamera() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("Vision");
    inst.startClientTeam(5171);
  }

  public Number[] getHoopCenter() {
    Number[] default_hoop_center_coord = new Number[] {0,0,0};
    hoop_coord = table.getEntry("translation_vector").getNumberArray(default_hoop_center_coord);
    x = hoop_coord[0].doubleValue();
    y = hoop_coord[1].doubleValue();
    z = hoop_coord[2].doubleValue();
    return hoop_coord;
  }

  public double getAngle(double v) {
    double v_squared = Math.pow(v, 2);
    double v_fourth = Math.pow(v, 4);
    double z_squared = Math.pow(z, 2);
    double g = 9.8;
    double var_square_root = v_fourth - g * (g * z_squared + 2 * y * v_squared);
    double equation = (v_squared + Math.sqrt(var_square_root)) / (g * z);
    double angle = Math.atan(equation);
    return angle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}