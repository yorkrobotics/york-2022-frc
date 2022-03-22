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

  // returns the needed angle of the shooter when given a particular initial velocity
  public double getAngle(double v, double shooter_angle) {
    double h_h = 5.0;// hoop height (feet)
    double h_r2g = 0.5; // circle center point to ground
    shooter_angle = shooter_angle / 180 * Math.PI;
    double shooter_radius = 1.5;

    double x_field = Math.sin(shooter_angle) * Math.tan(shooter_angle) * z + Math.cos(shooter_angle) * z;
    double y_field = h_h - shooter_radius * Math.sin(shooter_angle) - h_r2g;

    double v_squared = Math.pow(v, 2);
    double v_fourth = Math.pow(v, 4);
    double x_field_squared = Math.pow(x_field, 2);
    double g = 32.2 * 12;
    double var = v_fourth - g * (g * x_field_squared + 2 * y_field * v_squared); // can be used as determinant to get the minimum velocity required
    double equation = (v_squared + Math.sqrt(var)) / (g * x_field);
    double angle = Math.atan(equation);

    return angle;
  }

  // returns the needed velocity to shoot the target at the current angle
  public double calcVelocity(double theta) {
    double h_h = 5.0;// hoop height (feet)
    double h_r2g = 0.5; // circle center point to ground
    double shooter_angle = theta / 180 * Math.PI;
    double shooter_radius = 1.5;

    double x_field = Math.sin(shooter_angle) * Math.tan(shooter_angle) * z + Math.cos(shooter_angle) * z;
    double y_field = h_h - shooter_radius * Math.sin(shooter_angle) - h_r2g;

    SmartDashboard.putNumber("x_field", x_field);
    SmartDashboard.putNumber("y_field", y_field);

    double g = 32.2 * 12;
    theta = theta / 180 * Math.PI;
    double v_0 = Math.sqrt(Math.pow(x_field, 2) * g/(x_field*Math.sin(2*theta) - 2*y_field*Math.pow(Math.cos(theta), 2)));
    double motor_power = (v_0 - 132.9) / 242.48 ;
    if (motor_power > 1) {
      motor_power = 1;
    } else if (motor_power < 0) {
      motor_power = 0;
    }

    return motor_power;
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
