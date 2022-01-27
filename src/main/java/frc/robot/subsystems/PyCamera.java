// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;


public class PyCamera extends SubsystemBase {
  /** Creates a new PyCamera. */
  public PyCamera() {

    // for (NetworkTableEntry i : entries ) { // System.out.println("X: " + i);
    //   System.out.println(i.getName());
    // }

  }

  @Override
  public void periodic() {
  }

  public Number[] getHoopCenter (){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTableEntry[] entries = inst.getEntries("", 0);
    NetworkTable table = inst.getTable("Vision");
    //NetworkTable subtable = table.getSubTable("rPi Camera 0");
    inst.startClientTeam(5171);
    Number[] default_x = new Number[]{1,2};
    Number[] default_y = new Number[]{3,4};
    Number[] default_hoop_center_coord = new Number[]{0, 0, 0};
    Number[] x = table.getEntry("target_x").getNumberArray(default_x);
    Number[] hoop_coord = table.getEntry("translation_vector").getNumberArray(default_hoop_center_coord);
    //Number[] y = yEntry.getNumberArray(default_y);
    //Boolean bool = entry.getBoolean(default_bool);
    //System.out.println(bool);

    // System.out.println("-----------------------X values---------------- ");
    // for (Number i : x) { // System.out.println("X: " + i);
    //   System.out.println("x value: " + i);
    // }

    System.out.println("-----------------------Hoop Coord ---------------- ");
    for (Number i : hoop_coord) { // System.out.println("X: " + i);
      System.out.println(i);
    }
    return hoop_coord;
  }
}
