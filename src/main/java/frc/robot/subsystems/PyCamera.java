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
import org.opencv.core.*;
import frc.robot.Constants;

import java.io.File;
import java.lang.Double;
import java.util.ArrayList;
import java.util.concurrent.Flow.Subscriber;

public class PyCamera extends SubsystemBase {
  
  private static final PyCamera mPyCamera = new PyCamera();
  public static PyCamera getInstantce() {return mPyCamera;}

  public Number[] hoop_coord;
  public double x = 0;
  public double last_x;
  public double last_y;
  public double last_z;
  public double y = 0;
  public double z = 0;
  public double filteredAngle = 0;

  public double h_h = 106.0;// hoop height (inches)
  public double h_r2g = 6; // circle center point to ground
  public double shooter_radius = 1.5;

  public double x_field;
  public double y_field;
  public double last_x_field;
  public double last_y_field;
  
  public double towerAngle;
  public boolean isNaN = false;

  public double powerFudge = 0.98;

  Number[] default_hoop_center_coord = new Number[] {0,0,0};
  Number[] default_cam_vec = new Number[] {0,0,0};
  NetworkTable table;
  public LinearFilter filter = LinearFilter.singlePoleIIR(0, 0.02);
  
  public ArrayList<VisionSubscriber> subscribers = new ArrayList<VisionSubscriber>();
  
  /** Creates a new PyCamera. */
  public PyCamera() {

    NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
    table = inst.getTable("Vision");
    inst.startClientTeam(5171);

    SmartDashboard.putNumber("Power Fudge", 1);
    SmartDashboard.putNumber("Angle Fudge", 1);
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
  public double getAngle(double v) {
    double towerAngleRad = towerAngle / 180 * Math.PI;

    double x_field = Math.sin(towerAngleRad) * Math.tan(towerAngleRad) * z + Math.cos(towerAngleRad) * z;
    double y_field = h_h - shooter_radius * Math.sin(towerAngleRad) - h_r2g;

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
  // public double calcVelocity() {
  //   double shooter_angle = towerAngle / 180 * Math.PI;

  //   double x_field = Math.sin(shooter_angle) * Math.tan(shooter_angle) * z + Math.cos(shooter_angle) * z;
  //   double y_field = h_h - shooter_radius * Math.sin(shooter_angle) - h_r2g;

  //   double g = 32.2 * 12;
  //   double towerAngleRad = towerAngle / 180 * Math.PI;
  //   double v_0 = Math.sqrt(Math.pow(x_field, 2) * g/(x_field*Math.sin(2*towerAngleRad) - 2*y_field*Math.pow(Math.cos(towerAngleRad), 2)));
  //   double motor_power = (v_0 - 132.9) / 242.48 ;
  //   if (motor_power > 1) {
  //     motor_power = 1;
  //   } else if (motor_power < 0) {
  //     motor_power = 0;
  //   }

  //   return motor_power;
  // }

  public double getHorizontalAngle() {

    return filteredAngle;
  }

  public double getFieldX() {
    return x_field;
  }

  public double getFieldY() {
    return y_field;
  }

  // public double getFieldX(Rotation2d currentHeading) {
  //   double fieldX = z * Math.sin(currentHeading.getRadians());
  //   fieldX = Constants.FIELD_CENTER_X - fieldX;
  //   return fieldX;
  // }

  // public double getFieldY(Rotation2d currentHeading) {
  //   double fieldY = z * Math.cos(currentHeading.getRadians());
  //   fieldY = Constants.FIELD_CENTER_Y - fieldY;
  //   return fieldY;
  // }
  public void subscribe(VisionSubscriber subscriber) {
    subscribers.add(subscriber);
  }

  public void update() {
    for (VisionSubscriber subscriber: subscribers) {
      subscriber.handleNewValue(x_field);
    }
  }

  public boolean isNaN() {
    return isNaN;
  }

  public double calcVelocity() {
    double power = 37.77 + 0.0723 * x_field;
    return power * powerFudge;
  }

  public double calcAngle() {
    double angle = 67.62 - 0.04615 * x_field;
    return angle;
  }

  @Override
  public void periodic() {
    towerAngle = SmartDashboard.getNumber("Tower angle", 0);
    Tower mTower = Tower.getInstance();
    double theAngle = (mTower.getTowerAngle() - 30) / 180 * Math.PI;

    double[][] rRc = {
      {1,0,0},
      {0, Math.cos(theAngle),-Math.sin(theAngle)},
      {0, Math.sin(theAngle),Math.cos(theAngle)},
    };

    Number[] cL = table.getEntry("cam vec").getNumberArray(default_cam_vec);
    double rLx = 0, rLy = 0, rLz = 0;
    for (int j = 0; j < 3; j++) {
      rLx += rRc[0][j] * (double) cL[j];
      rLy += rRc[1][j] * (double) cL[j];
      rLz += rRc[2][j] * (double) cL[j];
    }

    double[] rL = {rLx,rLy,rLz};
    SmartDashboard.putNumber("L x", rL[0]);
    SmartDashboard.putNumber("L y", rL[1]);
    SmartDashboard.putNumber("L z", rL[2]);
   
    double[] rLo = {0,-28,-6};

    double[] p = {0, -104, 0};
    double[] n = {0, -1, 0};
    
    double[] diff = new double[3];
    for (int i = 0; i < 3; i++) {
      diff[i] = p[i] - rLo[i];
    }
    
    double numerator = 0;
    for (int i = 0; i < 3; i++) {
      numerator += diff[i] * n[i];
    }
 
    double denominator = 0;
    for (int i = 0; i < 3; i++) {
      denominator += rL[i] * n[i];
    }

    double quotient = numerator / denominator;

    double[] product = new double[3];
    for (int i = 0; i < 3; i++) {
      product[i] = rL[i] * quotient;
    }

    double[] points = new double[3];
    for (int i = 0; i < 3; i++) {
      points[i] = rLo[i] + product[i];
    }
    x = points[0];
    y = points[1];
    z = points[2];
    
    //points = rLo + rL * quotient;

     if (!(Double.valueOf(x).isNaN() || z == 0)) {
      double angle = Math.atan(x / z) / Math.PI * 180;
      if (!Double.valueOf(angle).isNaN()) {
        filteredAngle = angle;
      }
    } 

    if (Double.valueOf(x).isNaN() || Double.valueOf(z).isNaN() || Double.valueOf(y).isNaN()) {
      isNaN = true;
    }


    if (mTower.isHome()) {
      double shooter_angle = towerAngle / 180 * Math.PI;
      y_field = h_h - shooter_radius * Math.sin(shooter_angle) - h_r2g;
      // x_field = Math.tan(shooter_angle) * (Math.sin(shooter_angle) *  z - y_field) + Math.cos(shooter_angle) * z;
      x_field = Math.sqrt(x * x + z * z);
    }

    if (last_x != x || last_y != y || last_z != z) {
        // this.subscribe(subscribers);
        this.update();
    }

    last_x = x;
    last_y = y;
    last_z = z;

    last_x_field = x_field;
    last_y_field = y_field;

    // double shooter_angle = towerAngle / 180 * Math.PI;
    // y_field = h_h - shooter_radius * Math.sin(shooter_angle) - h_r2g;
    // x_field = Math.tan(shooter_angle) * (Math.sin(shooter_angle) *  z - y_field) + Math.cos(shooter_angle) * z;
    // x_field = filter.calculate(x_field); // TODO: to be tested

    SmartDashboard.putNumber("x", x);
    SmartDashboard.putNumber("y", y);
    SmartDashboard.putNumber("z", z);
    SmartDashboard.putNumber("filterdAngle", filteredAngle);
    // filteredAngle = filter.calculate(angle);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Field X", x_field);
    SmartDashboard.putNumber("Field y", y_field);


    /* double velocity = this.calcVelocity();
    SmartDashboard.putNumber("shooter velocity: ", velocity); */
   
  }
}
