// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.Constants;

import java.io.File;
import java.lang.Double;


public class PyCamera extends SubsystemBase {
  public Number[] hoop_coord;
  public double x = 0;
  public double y = 0;
  public double z = 0;
  public double filteredAngle = 0;
  Number[] default_hoop_center_coord = new Number[] {0,0,0};
  NetworkTable table;
  // public LinearFilter filter = LinearFilter.singlePoleIIR(0, 0.02);
  
  /** Creates a new PyCamera. */
  public PyCamera() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
    table = inst.getTable("Vision");
    inst.startClientTeam(5171);
    // hoop_coord = table.getEntry("translation_vector").getNumberArray(default_hoop_center_coord);
    // table.addEntryListener("translation_vector", (tbl, key, entry, value, flags) -> {
    //   double[] hoop_coord = value.getDoubleArray();
    //   if (!(Double.valueOf(hoop_coord[0]).isNaN() || Double.valueOf(hoop_coord[1]).isNaN() || Double.valueOf(hoop_coord[2]).isNaN())) {
    //     x = hoop_coord[0];
    //     y = hoop_coord[1];
    //     z = hoop_coord[2];
    //   }
    //   System.out.println("entry listener executed");
    // }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  }

  public Number[] getHoopCenter() {
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

  public double getHorizontalAngle() {

    return filteredAngle;
  }

  public double getFieldX(Rotation2d currentHeading) {
    double fieldX = z * Math.sin(currentHeading.getRadians());
    fieldX = Constants.FIELD_CENTER_X - fieldX;
    return fieldX;
  }

  public double getFieldY(Rotation2d currentHeading) {
    double fieldY = z * Math.cos(currentHeading.getRadians());
    fieldY = Constants.FIELD_CENTER_Y - fieldY;
    return fieldY;
  }

  @Override
  public void periodic() {
    if (!(Double.valueOf(x).isNaN() || z == 0)) {
      double angle = Math.atan(x / z) / Math.PI * 180;
      filteredAngle = angle;
    }

    hoop_coord = table.getEntry("translation_vector").getNumberArray(default_hoop_center_coord);
    x = hoop_coord[0].doubleValue();
    y = hoop_coord[1].doubleValue();
    z = hoop_coord[2].doubleValue();
    SmartDashboard.putNumber("filterdAngle", filteredAngle);
    // filteredAngle = filter.calculate(angle);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("z", z);
    SmartDashboard.putNumber("filteredAngle", filteredAngle);
  }
}
