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
  public PyCamera() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTableEntry[] entries = inst.getEntries("", 0);
    NetworkTable table = inst.getTable("Vision");
    NetworkTableEntry xEntry = table.getEntry("target_x");
    NetworkTableEntry yEntry = table.getEntry("target_y");
    inst.startClientTeam(5171);
    while (true) {
      try {
        Thread.sleep(1000);
      } catch (InterruptedException ex) {
        System.out.println("interrupted");
        return;
      }
      Number[] default_x = new Number[]{1,2};
      Number[] default_y = new Number[]{3,4};
      Number[] x = xEntry.getNumberArray(default_x);
      Number[] y = yEntry.getNumberArray(default_y);
      for (NetworkTableEntry i : entries ) {
        // System.out.println("X: " + i);
        System.out.println(i.getName());
      }
    }

  }
}
